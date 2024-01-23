#include "DataLogger.h"

#include <fstream>
#include <sstream>
#include <iostream>



DataLogger::DataLogger(std::string homePath, std::string filePath) {
    fileStream_.open(homePath + "\\data\\DataLogs\\" + filePath + ".csv");
    fileStream_ << "Rot0,Rot1,Rot2,Tran0,Tran1,Tran2,PoseValid,FrameNumber,EstimationTime" << std::endl;
}


DataLogger::~DataLogger() {
    fileStream_.close();
}


void DataLogger::logData(cv::Mat rVec, cv::Mat tVec, bool poseValid, size_t frameNumber, uint32_t estimationTime) {

    fileStream_ << rVec.at<float>(0,0) << "," << rVec.at<float>(0,1) << "," << rVec.at<float>(0,2) << "," << tVec.at<float>(0,0) << "," << tVec.at<float>(0,1) << "," << tVec.at<float>(0,2) << "," << poseValid << "," << frameNumber << "," << estimationTime << std::endl;

}