#ifndef SATELLITE_CLASS_H
#define SATELLITE_CLASS_H


#include "stdint.h"

#include "LedClass.h"

#include <opencv2/opencv.hpp>


/**
 * @brief Class to contain all satellite important data e.g. Pose, velocity LED model etc.
 * 
 */
class Satellite {
public:


    ///Current pose of satellite
    cv::Point3f rVec;
    cv::Point3f tVec;

    ///Current velocity of satellite
    cv::Point3f tVel;
    ///Current angular velocity of satellite
    cv::Point3f rVel;

    ///Model for led IDs and Positions in local coordinates of satellite
    std::vector<LED> ledModel;


    /**
     * @brief Construct a new Sat Model object using known led model and initialpose
     * 
     * @param ledModel vector containing all led positions in reference to satellite coordinate frame and their IDs
     */
    Satellite(const std::vector<LED> ledModel);



};





#endif