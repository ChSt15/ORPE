#ifndef LED_CLASS_H
#define LED_CLASS_H


#include "stdint.h"

#include <opencv2/opencv.hpp>


class LED {
private:

    cv::Point3f p_;
    cv::Point3f v_;

    int32_t id_ = 0;

    bool lastState_ = true;
    int64_t lastStateTime_ = 0;
    int64_t lastPosSet_ = 0;

    bool state_ = false;

    bool wasOff_ = false;

    int32_t currentBit_ = -1;
    int32_t numberOnBits_ = 0;
    uint32_t code_ = 0;


public:

    LED(int32_t id = 0);

    LED(cv::Point2f point, int32_t id = 0);

    LED(cv::Point3f point, int32_t id = 0);

    LED(float x, float y, float z = 0, int32_t id = 0);


    /**
     * @brief Sets the current object state and runs algorithm for id detection
     * 
     * @param state 
     */
    void setState(bool state, size_t currentFrameTime);

    void setPos(cv::Point3f point, bool updateTime = false, size_t currentFrameTime = 0);

    void setVel(cv::Point3f point);

    void setId(int32_t id);


    int64_t getLastStateChange();

    int64_t getLastPosChange();

    int32_t getId() const;

    cv::Point3f getPos() const;

    cv::Point3f getVel() const;

    bool getState() const;

    LED getPredictionAtTime(int64_t time);


    void drawLED(cv::Mat& image) const;


};





#endif