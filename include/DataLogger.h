#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H


#include "stdint.h"

#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>



class DataLogger {
private:

    std::ofstream fileStream_;

    std::string filePath_;


public:

    DataLogger(std::string homePath, std::string filePath);

    ~DataLogger();

    /**
     * @brief Writes the given data into a new line inside the logging file
     * 
     * @param rVec 
     * @param tVec 
     * @param poseValid 
     * @param frameNumber 
     * @param estimationTime Amount of time taken to estimate pose for this frame in us
     */
    void logData(cv::Mat rVec, cv::Mat tVec, bool poseValid, size_t frameNumber, uint32_t estimationTime);

};



#endif