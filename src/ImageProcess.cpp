#include "ImageProcess.h"

#include <vector>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;




vector<Point2f> findPointsInBitmap(Mat& grayScaleImage, Mat& binaryImage) {

    static bool wasAllocated = false;
    static uint16_t** labelledImage;

    if (!wasAllocated) {
        wasAllocated = true;

        labelledImage = new uint16_t*[binaryImage.rows];
        for (size_t i = 0; i < binaryImage.rows; i++) labelledImage[i] = new uint16_t[binaryImage.cols];

    }

    //The following algorithm is similar to those in star trackers. Regions will be first labelled, then the weighted average of each region will be calclulated.
    std::vector<Point2f> points;
    std::vector<uint32_t> pointsPixelCount;

    //Initialise image array
    for (int col = 0; col < binaryImage.cols; col++) {
        for (int row = 0; row < binaryImage.rows; row++) {
            labelledImage[row][col] = binaryImage.at<uchar>(row, col) == 255 ? 1:0;
        }
    }


    const uint16_t offsetLabel = 2;
    uint16_t pointCounter = offsetLabel;

    //Region labelling
    for (int row = 1; row < binaryImage.rows; row++) {

        for (int col = 1; col < binaryImage.cols - 1; col++) {

            if (labelledImage[row][col] == 1) {

                if (labelledImage[row][col - 1] > 1)
                {
                    labelledImage[row][col] = labelledImage[row][col - 1];
                }
                else if (labelledImage[row - 1][col - 1] > 1)
                {
                    labelledImage[row][col] = labelledImage[row - 1][col - 1];
                }
                else if (labelledImage[row - 1][col] > 1)
                {
                    labelledImage[row][col] = labelledImage[row - 1][col];
                }
                else if (labelledImage[row - 1][col + 1] > 1)
                {
                    labelledImage[row][col] = labelledImage[row - 1][col + 1];
                }
                else
                {
                    labelledImage[row][col] = pointCounter;
                    points.push_back(Point2f(0,0));
                    pointsPixelCount.push_back(0);
                    pointCounter++;
                }

                Point2f& point = points.at(labelledImage[row][col] - offsetLabel); // 2 subtracted due to labell offset
                pointsPixelCount.at(labelledImage[row][col] - offsetLabel)++; //Increment count

                //Sum up the position
                point.x += col;
                point.y += row;

            }
            
        }

    }

    for (size_t p = 0; p < points.size(); p++) {

        Point2f& point = points.at(p);
        auto count = pointsPixelCount.at(p);
        point.x /= count;
        point.y /= count;

    }

    return points;

}



vector<Point2f> getPointsInImage(Mat& image, int thresholdMin) {

    vector<Point2f> pointsImage;

    Mat showIm;
    //cv::resize(image, showIm, cv::Size(1280, 720));
    //cv::imshow("Original", showIm);

    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    //cv::imshow("Grayscale", gray);

    Mat blur;
    GaussianBlur(gray, blur, cv::Size(5, 5), 0);

    //cv::imshow("Blur", blur);

    Mat bw;
    double threshVal = cv::threshold(blur, bw, 0, 255, THRESH_TOZERO | THRESH_OTSU);
    if (threshVal < thresholdMin) cv::threshold(blur, bw, thresholdMin, 255, THRESH_TOZERO);

    //cv::adaptiveThreshold(gray, bw, 255, cv::AdaptiveThresholdTypes::ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 9, 7);
    //cv::resize(bw, showIm, cv::Size(1280, 720));
    //cv::imshow("Binary", showIm);

    //Mat canny;
    //Canny(blur, canny, 100, 110);

    //cv::imshow("Canny", canny);


    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(bw, contours, hierarchy, RetrievalModes::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size() && pointsImage.size() < 50; i++ ) {

        //if (!isContourConvex(contours[i])) {

            Moments M = moments(contours[i]);
            Point2f c(M.m10/M.m00, M.m01/M.m00);

            if (!isnan(c.x) && !isnan(c.y) && !isinf(c.x) && !isinf(c.y)) pointsImage.push_back(c);

            //drawContours(bw, contours, i, Scalar(0,255,0));

        //}

    }

    //drawContours(image, contours, -1, Scalar(0,255,0));

    //imshow("Test", image);

    //waitKey(0);

    // Setup SimpleBlobDetector parameters.
    /*SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 1;
    params.maxThreshold = 50;

    // Filter by Area.
    params.filterByArea = false;
    params.minArea = 1;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.2;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.2;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.2;

    std::vector<KeyPoint> keypoints;

    // Set up detector with params
    static Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Detect blobs
    detector->detect(bw, keypoints);

    //Mat im_with_keypoints;
    //drawKeypoints(image, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    for (auto const& p:keypoints) {

        Point2f c = p.pt;

        if (!isnan(c.x) && !isnan(c.y) && !isinf(c.x) && !isinf(c.y)) pointsImage.push_back(c);

    }*/

    //imshow("test", im_with_keypoints);

    //waitKey(0);

    //pointsImage = findPointsInBitmap(blur, bw);

    return pointsImage;

}

