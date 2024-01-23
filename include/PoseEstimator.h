#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <opencv2/opencv.hpp>

#include "Predictor.h"
#include "SatelliteClass.h"
#include "LedClass.h"

/**
 * @brief   This class is used to estimate the position of an object with LEDs using a stream of frames and a model of the LEDs position and ID.
 *          The estimator takes care of image Processing, LED tracking and decoding, solving the PnP-Problem and determining if the estimation is valid.
 *
 * @note  - For usage, the giveEstimatorNextImage(image) should be called first and given the unprocessed image from the camera. Then estimatePose() must be called
 *          and then getPoseEstimation(pose) to retrieve the pose value written into pose parameter. The return value of getPosegetPoseEstimation() must be verified
 *          to be true, otherwise the pose estimation is invalid and should not be used.
 *        - giveEstimatorNextImage() must be called and passed images at exactly the correct frame rate as prediction and LED decoding assumes and relies on this.
 *
 *
 */
class PoseEstimator
{
private:
    Predictor predictor_;
    Satellite satellite_;

    cv::Mat cameraMatrix_;
    cv::Mat cameraDistorionMatrix_;

    size_t ledCodingRate_ = 1;

    cv::Mat currentImage_;
    size_t currentImageNumber_ = 0;
    bool imageIsNew_ = false;
    bool poseEstimationValid_ = false;
    bool outputPoseEstimationValid_ = false;

    uint8_t dynamicThres_ = 20;

    std::vector<LED> leds_;

    // Counts the number of pose estimations in a row. Decrements on failed estimation.
    size_t estimationCounter_ = 0;

    // Pose data from last estimation
    cv::Point3f lastRot_;
    cv::Point3f lastTran_;

public:
    /**
     * @brief Construct a new Pose Estimator object
     *
     * @param ledCodingRate Number of frames per LED coding bit.
     * @param ledModel Vector containing all led positions of 3D space in reference to local satellite coordinate system.
     */
    PoseEstimator(std::vector<LED> ledModel, cv::Mat cameraMatrix, cv::Mat cameraDistorionMatrix);

    /**
     * @brief Give estimator new image to work with. No Calculations will be done in this function.
     * @see estimatePose()
     *
     * @param image
     */
    void giveEstimatorNextImage(cv::Mat image);

    /**
     * @brief Uses most recent image to get an estimation for pose.
     * @note This function does the bulk of calculations and therefore takes a considerable amount of time.
     *
     */
    void estimatePose();

    /**
     * @brief Get the Pose Estimation object
     * 
     * @param rot Reference to Point3f object to receive rotation data in rodrigues format.
     * @param tran Reference to Point3f object to receive position data. Unit is the same as the given led model units.
     * @return true Estimation is valid.
     * @return false Estimation is a prediction and not valid.
     */
    bool getPoseEstimation(cv::Point3f &rot, cv::Point3f &tran);

    /**
     * @brief Get all current Points (including identified LEDs) detected in image.
     *
     * @return std::vector<LED>
     */
    std::vector<LED> getCurrentPoints();

    /**
     * @brief Get the Current Frame Number
     *
     * @return size_t
     */
    size_t getCurrentFrameNumber();

    /**
     * @brief Resets all internal values. Should be called when object not in use to keep counter values from overflowing.
     *
     * @note Will stop all tracking. If calling during tracking, all LEDs must first be decoded again causing delay before Pose estimation is valid.
     *
     */
    void reset();
};

#endif
