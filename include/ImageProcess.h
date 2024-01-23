#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H


#include <vector>
#include <opencv2/opencv.hpp>


std::vector<cv::Point2f> getPointsInImage(cv::Mat& image, int thresholdMin = 5);


#endif