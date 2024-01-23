#include "PoseEstimator.h"

#include "LedProcess.h"
#include "ImageProcess.h"


PoseEstimator::PoseEstimator(std::vector<LED> ledModel, cv::Mat cameraMatrix, cv::Mat cameraDistorionMatrix) : satellite_(ledModel),
                                                                                                                                     cameraMatrix_(cameraMatrix),
                                                                                                                                     cameraDistorionMatrix_(cameraDistorionMatrix) {}

void PoseEstimator::giveEstimatorNextImage(cv::Mat image)
{

    imageIsNew_ = true;
    currentImage_ = image;
    currentImageNumber_++;
}

bool PoseEstimator::getPoseEstimation(cv::Point3f &rot, cv::Point3f &tran)
{

    bool returnVal = imageIsNew_ && outputPoseEstimationValid_;

    rot = satellite_.rVec;
    tran = satellite_.tVec;

    imageIsNew_ = false;

    return returnVal;
}

std::vector<LED> PoseEstimator::getCurrentPoints()
{
    return leds_;
}

size_t PoseEstimator::getCurrentFrameNumber()
{
    return currentImageNumber_;
}

void PoseEstimator::reset()
{

    currentImageNumber_ = 0;
    leds_.clear();

    poseEstimationValid_ = false;
    imageIsNew_ = false;
}

void PoseEstimator::estimatePose()
{

    if (!imageIsNew_)
        return; // Do not estimate if image is not new

    if (currentImage_.empty())
        return; // Image is empty. Leave.

    // Resize image to known value.
    //cv::resize(currentImage_, currentImage_, cv::Size(1920, 1080));

    // Get prediction
    std::vector<LED> predictedPoints = leds_;
    auto Predbeg = std::chrono::steady_clock::now();
    if (poseEstimationValid_) //3D space prediction of positive ID points
    {

        cv::Point3f rPred = satellite_.rVec + satellite_.rVel;
        cv::Point3f tPred = satellite_.tVec + satellite_.tVel;

        auto modelPointsPred = predictPointsFromModel(satellite_.ledModel, rPred, tPred, cameraMatrix_, cameraDistorionMatrix_);

        //predictedPoints = modelPointsPred;

        for (auto const &p : modelPointsPred)
        {

            for (auto &i : predictedPoints)
            {

                if (i.getId() == p.getId())
                {

                    i.setPos(p.getPos());
                    break;
                }
            }
        }
    }

    for (auto& p: predictedPoints) //Image plane prediction of negative ID points or if estimation was invalid
    {

        if (p.getId() < 0 || !poseEstimationValid_) p = p.getPredictionAtTime(currentImageNumber_); 

    }


    auto imageDetBeg = std::chrono::steady_clock::now();
    // Use thresholding and point identifying to get all points in image.
    auto pointsImage = getPointsInImage(currentImage_, dynamicThres_);

    //If we have 50 points in the image, then the threshold is not high enough. Adjust threshold to limit points
    if (leds_.size() > satellite_.ledModel.size() * 2 && dynamicThres_ < 250) {
	dynamicThres_++;
    } else if (leds_.size() < satellite_.ledModel.size() && dynamicThres_ > 5) {
	dynamicThres_--;
    }



    auto imageMatBeg = std::chrono::steady_clock::now();
    // Match the points in current image to known LEDs
    leds_ = matchNewPoints(pointsImage, leds_, predictedPoints, currentImageNumber_);

    // Remove LEDs with multiple copies (Multiple LEDs with same ID)
    std::vector<LED> validImagePoints, validModelPoints;
    getValidModelPoints(getValidPoints(leds_), satellite_.ledModel, validImagePoints, validModelPoints);

    // Solve the PnP problem if more than 3 LEDs have been identified.
    bool solveOk = false;
    cv::Point3f rot, tran;
    if (validModelPoints.size() >= 4)
    {

        cv::Mat rotM, tranM;
        cv::Mat(satellite_.rVec).copyTo(rotM); //Use old data to help iterative solver
        cv::Mat(satellite_.tVec).copyTo(tranM); //Use old data to help iterative solver
        solveOk = cv::solvePnPRansac(cv::Mat(convertLEDTo3dPoint(validModelPoints)), cv::Mat(convertLEDTo2dPoint(validImagePoints)), cameraMatrix_, cameraDistorionMatrix_, rotM, tranM, true, 100, 8.0, 0.99, cv::noArray(), poseEstimationValid_ ? cv::SolvePnPMethod::SOLVEPNP_IPPE : cv::SolvePnPMethod::SOLVEPNP_IPPE/*SOLVEPNP_EPNP*/);
        //solveOk = cv::solvePnP(cv::Mat(convertLEDTo3dPoint(validModelPoints)), cv::Mat(convertLEDTo2dPoint(validImagePoints)), cameraMatrix_, cameraDistorionMatrix_, rotM, tranM, false, cv::SolvePnPMethod::SOLVEPNP_EPNP);
        rot = cv::Point3f(rotM);
        tran = cv::Point3f(tranM);

        float rotDiff = (rot - satellite_.rVec).dot(rot - satellite_.rVec); //Square sum of difference between prediction and estimation
        float tranDiff = (tran - satellite_.tVec).dot(tran - satellite_.tVec);

        //std::cout << "Difference: " << difference << std::endl;

        // Check for output errors.
        float angle = sqrt(rot.dot(rot));
        if (angle > 2 * 3.1416)
            solveOk = false;
        else if (isnan(tran.x) || isnan(tran.y) || isnan(tran.z) || isinf(tran.x) || isinf(tran.y) || isinf(tran.z))
            solveOk = false;
        else if (isnan(rot.x) || isnan(rot.y) || isnan(rot.z) || isinf(rot.x) || isinf(rot.y) || isinf(rot.z))
            solveOk = false;

        else if ((sqrt(rotDiff) > 10*3.14/180 || sqrt(tranDiff) > 30) && estimationCounter_ > 0) //If difference between prediction and estimation is greater than 5 deg or 2 cm and there has been a good solve
            solveOk = false;

    }
    


    // Convert solvePnP output and Predict LED positions of LEDs that are off to help with tracking on next frame
    if (solveOk)
    {

        estimationCounter_++;
        if (estimationCounter_ > 20)
            estimationCounter_ = 20;

        if (!poseEstimationValid_)
        { // First ok solve. Change velocities to 0.

            satellite_.tVel = satellite_.rVel = cv::Point3f(0, 0, 0);
        }
        else
        {

            satellite_.rVel = rot - satellite_.rVec; // Change per frame
            satellite_.tVel = tran - satellite_.tVec;

        }

        satellite_.rVec = rot;
        satellite_.tVec = tran;
    }
    else
    {

        estimationCounter_ /= 2;
        // if (estimationCounter_ > 0) estimationCounter_--;

        // extrapolate state to current frame
        satellite_.rVec = satellite_.rVec + satellite_.rVel;
        satellite_.tVec = satellite_.tVec + satellite_.tVel;
    }

    // Do a check to see if newest pose estimation is possible and valid.
    poseEstimationValid_ = estimationCounter_ > 2;
    outputPoseEstimationValid_ = poseEstimationValid_ && solveOk;

    if (outputPoseEstimationValid_) {

	//Project model points into image and do a quick led identification
        if (validModelPoints.size() > 5) {
       	    auto proj = predictPointsFromModel(satellite_.ledModel, rot, tran, cameraMatrix_, cameraDistorionMatrix_);
      	    leds_ = quickLEDIdent(leds_, proj);
	}

    }

}
