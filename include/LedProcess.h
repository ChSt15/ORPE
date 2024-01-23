#ifndef LED_PROCESS_H
#define LED_PROCESS_H


#include "LedClass.h"

#include <vector>
#include <opencv2/opencv.hpp>


/**
 * @brief Converts the vector of LEDs into their 2D position.
 * 
 * @param leds 
 * @return std::vector<cv::Point2f> 
 */
std::vector<cv::Point2f> convertLEDTo2dPoint(std::vector<LED> leds);

/**
 * @brief Converts the vector of LEDs into their §D position.
 * 
 * @param leds 
 * @return std::vector<cv::Point3f> 
 */
std::vector<cv::Point3f> convertLEDTo3dPoint(std::vector<LED> leds);

/**
 * @brief Uses model, pose and image information to quickly identify any visible model points using their position.
 * @note return list contains image points with any updated indentified image points
 * 
 * @param imagePoints All points from the image.
 * @param projectedModelPoints Points from the model projected into the image
 * @return std::vector<LED> updated list of all image points.
 */
std::vector<LED> quickLEDIdent(std::vector<LED> imagePoints, std::vector<LED> projectedModelPoints);

/**
 * @brief Matches the points from a new image to the currently tracked points and updated their position and velocity
 * 
 * @param newPoints 
 * @param oldPoints 
 * @param predictedPoints 
 * @param currentFrameTime 
 * @return std::vector<LED> 
 */
std::vector<LED> matchNewPoints(std::vector<cv::Point2f> newPoints, std::vector<LED> oldPoints, std::vector<LED> predictedPoints, size_t currentFrameTime);

/**
 * @brief Created a vector from the given LEDs conataining only the LEDs with positive IDs and LEDs that are currently turned On and only 1 with their ID value. Basically leaves only the LEDs to be used as image points for the PnP-Solver.
 * 
 * @param points 
 * @return std::vector<LED> 
 */
std::vector<LED> getValidPoints(std::vector<LED> points);

/**
 * @brief Takes the given model and image points and places only the points into the valid point containers with LEDs that have corresponding ID to each other. Basically creates 2 vectors containing the model and image points for the PnP-Solver.
 * 
 * @param imagePoints 
 * @param modelPoints 
 * @param validImagePoints 
 * @param validModelPoints 
 */
void getValidModelPoints(std::vector<LED> imagePoints, std::vector<LED> modelPoints, std::vector<LED>& validImagePoints, std::vector<LED>& validModelPoints);

/**
 * @brief Predicts new LED positions using velocity
 * 
 * @param oldPoints 
 * @param time 
 * @return std::vector<LED> 
 */
std::vector<LED> predictPoints(std::vector<LED> oldPoints, int64_t time);

/**
 * @brief Predicts missing LEDs (Off state) using known Camera Pose
 * 
 * @param rotation 
 * @param translation 
 * @param cameraMat 
 * @return std::vector<LED> 
 */
std::vector<LED> predictPointsFromModel(std::vector<LED> points, const cv::Point3f& rotation, const cv::Point3f& translation, cv::InputArray cameraMatrix, cv::InputArray distCoeffs);



#endif