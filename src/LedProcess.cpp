#include "LedClass.h"


using namespace std;

cv::Point2f to2d(cv::Point3f p) {
    return cv::Point2f(p.x, p.y);
}

cv::Point3f to3d(cv::Point2f p) {
    return cv::Point3f(p.x, p.y, 0.0);
}

template<typename TYPE>
float dist(TYPE a, TYPE b) {
    TYPE ab = a - b;
    return sqrt(ab.dot(ab));
}


std::vector<cv::Point2f> convertLEDTo2dPoint(std::vector<LED> leds) {

    std::vector<cv::Point2f> points;

    for (auto const& l: leds) points.push_back(to2d(l.getPos())); 

    return points;

}


std::vector<cv::Point3f> convertLEDTo3dPoint(std::vector<LED> leds) {

    std::vector<cv::Point3f> points;

    for (auto const& l: leds) points.push_back(l.getPos()); 

    return points;

}



std::vector<LED> predictPointsFromModel(std::vector<LED> points, const cv::Point3f& rotation, const cv::Point3f& translation, cv::InputArray cameraMatrix, cv::InputArray distCoeffs) {

    std::vector<LED> prediction;

    std::vector< cv::Point3f > axisPoints = convertLEDTo3dPoint(points);

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, cv::Mat(rotation), cv::Mat(translation), cameraMatrix, distCoeffs, imagePoints);

    for (uint32_t i = 0; i < points.size(); i++) {

        LED p = points[i];
        p.setPos(to3d(imagePoints[i]));

        prediction.push_back(p);

    }

    return prediction;

}


std::vector<LED> predictPoints(std::vector<LED> points, int64_t time) {

    std::vector<LED> prediction;

    for (auto& p: points) {

        cv::Point3f pos = p.getPos() + p.getVel()*float(time - p.getLastPosChange());

        p.setPos(pos, true);

        prediction.push_back(p);

    }

    return prediction;

}


std::vector<LED> quickLEDIdent(std::vector<LED> imagePoints, std::vector<LED> projectedModelPoints) {

    //Project the model points onto the image plane and then using the deviation of the distance between the projected points and the cooresponding image points, 
    //find any model points that are not in the image points and add update the matching image point with the model point ID.

    //Calculate the average distance between the projected model points and the image points
    float deviation = 0;
    size_t count = 0;
    for (uint32_t i = 0; i < imagePoints.size(); i++) {

        if (imagePoints[i].getId() < 1) continue; //Skip unidentified points (ID < 1)

        //Count number of points with the same ID
        size_t sameID = 0;
        for (uint32_t j = 0; j < imagePoints.size() && sameID <= 1; j++) {

            if (imagePoints[j].getId() == imagePoints[i].getId()) sameID++;

        }
        if (sameID > 1) continue; //Skip points with more than 1 point with the same ID

        //Find corresponding projected model point
        for (auto const& p: projectedModelPoints) {

            if (p.getId() == imagePoints[i].getId()) {

                auto dis = dist(to2d(projectedModelPoints[i].getPos()), to2d(imagePoints[i].getPos()));
                deviation += dis*dis;
                count++;
                break;

            }

        }

    }

    //We require at least 2 good points for calc deviation
    if (count < 2) return imagePoints;
    deviation = sqrt(deviation / (count - 1));

    //std::cout << "LED Quick Ident dev: " << deviation << std::endl;

    //Go through unidentified image points and find any projected model points that are within the deviation.
    for (auto& i: imagePoints) {

        if (i.getId() < 0) {
		
	    //Find the closest fitting model point
	    float closestDis = deviation;
	    auto closest = projectedModelPoints[0];
            for (auto const& p: projectedModelPoints) {

		bool hasMatch = false;
		for (auto const& j: imagePoints) { //If this model point has a fitting image point, then dont use this point
		    if (j.getId() == p.getId()) {
			hasMatch = true;
			break;
		    }
		}
		
		if (hasMatch) continue;

		//At this point we found a model point without a matching image point, we now check if its closer.
		
		auto dis = dist(to2d(p.getPos()), to2d(i.getPos()));

                if (dis < deviation && dis < closestDis) {
		    
		    closestDis = dis;
		    closest = p;

                }

            }

	    if (closestDis != deviation) { //Have we found the closest model point within the deviation and without a matching image point?

		i.setId(closest.getId());

	    }

        }

    }

    return imagePoints;

}


std::vector<LED> matchNewPoints(std::vector<cv::Point2f> newPoints, std::vector<LED> oldPoints, std::vector<LED> predictedPoints, size_t currentFrameTime) {

    std::vector<LED> combined;


    if (predictedPoints.size() == 0) { //In the case where no points need to be matched

        for (int32_t i = 0; i < newPoints.size(); i++) {

            combined.push_back(LED(newPoints[i], -i - 1));

        }

    } else {

        //std::vector<float> posChanges;
        std::vector<int32_t> pairs;
        for (auto const& p: newPoints) {

            int32_t closest = 0;

            for (uint32_t i = 0; i < predictedPoints.size(); i++) {

                if (dist(p, to2d(predictedPoints[i].getPos())) < dist(p, to2d(predictedPoints[closest].getPos()))) {
                    closest = i;
                }

            }

            //posChanges.push_back(dist(p, to2d(predictedPoints[closest].getPos())));

            pairs.push_back(closest);

        }

        /*float movementDisAvg = 0;
        float movementDisDev = 0;

        for (auto const& d: posChanges) {
            movementDisAvg += d;
        }
        movementDisAvg /= posChanges.size();

        for (auto const& d: posChanges) {

            float buf = d - movementDisAvg;

            movementDisDev += buf*buf;

        }

        movementDisDev = sqrt(movementDisDev/(posChanges.size()-1));*/


        for (uint32_t p = 0; p < predictedPoints.size(); p++) {

            int32_t closest = -1;

            for (int32_t i = 0; i < pairs.size(); i++) {

                if (pairs[i] == p) {

                    cv::Point2f movement = newPoints[i] - to2d(predictedPoints[p].getPos());

                    float dis = dist(movement, cv::Point2f());

                    //if (abs(movementDisAvg - dis) < movementDisDev) {

                        if (closest == -1) closest = i; //Found first corresponding new point.
                        else {
                        
                            if (dist(newPoints[i], to2d(predictedPoints[p].getPos())) < dist(newPoints[closest], to2d(predictedPoints[p].getPos()))) { 

                                int32_t nextID = -1;
                                for (;;nextID--) {

                                    bool idFound = true;

                                    for (auto const& p: predictedPoints) {

                                        if (p.getId() == nextID) {
                                            idFound = false;
                                            break;
                                        }

                                    }

                                    if (idFound) {
                                        
                                        for (auto const& p: combined) {

                                            if (p.getId() == nextID) {
                                                idFound = false;
                                                break;
                                            }

                                        }   

                                    }

                                    if (idFound) break;

                                }
                                combined.push_back(LED(newPoints[closest], nextID));

                                closest = i;

                            } else {

                                int32_t nextID = -1;
                                for (;;nextID--) {

                                    bool idFound = true;

                                    for (auto const& p: predictedPoints) {

                                        if (p.getId() == nextID) {
                                            idFound = false;
                                            break;
                                        }

                                    }

                                    if (idFound) {
                                        
                                        for (auto const& p: combined) {

                                            if (p.getId() == nextID) {
                                                idFound = false;
                                                break;
                                            }

                                        }   

                                    }

                                    if (idFound) break;

                                }

                                combined.push_back(LED(newPoints[i], nextID));

                            }

                        }

                    //}

                }

            }

            if (closest != -1) {

                float dT = currentFrameTime - oldPoints[p].getLastPosChange();

                cv::Point3f vel = to3d(newPoints[closest]) - oldPoints[p].getPos();
                vel = vel/dT;

                if (dT > 0 && vel == vel) predictedPoints[p].setVel(vel);
                predictedPoints[p].setPos(to3d(newPoints[closest]));
                predictedPoints[p].setState(true, currentFrameTime);

            } else predictedPoints[p].setState(false, currentFrameTime);

            if (!(!predictedPoints[p].getState() && currentFrameTime - predictedPoints[p].getLastStateChange() > 6)) combined.push_back(predictedPoints[p]);

        }

    }


    return combined;
    
}


std::vector<LED> getValidPoints(std::vector<LED> points) {

    std::vector<LED> validPoints;

    for (uint32_t i = 0; i < points.size(); i++) {

        bool moreThanOne = false;

        for (uint32_t j = 0; j < points.size(); j++) {

            if (i != j && points[i].getId() == points[j].getId()) {
                moreThanOne = true;
                break;
            }

        }

        if (!moreThanOne && points[i].getId() > 0 && points[i].getState()) validPoints.push_back(points[i]);

    }

    return validPoints;

}


void getValidModelPoints(std::vector<LED> imagePoints, std::vector<LED> modelPoints, std::vector<LED>& validImagePoints, std::vector<LED>& validModelPoints) {

    for (auto const& i: imagePoints) {

        LED const* modelPoint = nullptr;

        for (auto const& m: modelPoints) {

            if (m.getId() == i.getId()) {
                modelPoint = &m;
                break;
            }

        }

        if (modelPoint != nullptr) {
            validModelPoints.push_back(*modelPoint);
            validImagePoints.push_back(i);
        }

    }

}
