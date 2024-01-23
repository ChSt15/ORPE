#include "LedClass.h"

#include "bitset"


using namespace std;


LED::LED(int32_t id): p_(0,0,0), v_(0,0,0), id_(id) {}

LED::LED(cv::Point2f point, int32_t id): p_(point), v_(0,0,0), id_(id) {}

LED::LED(cv::Point3f point, int32_t id): p_(point), v_(0,0,0), id_(id) {}

LED::LED(float x, float y, float z, int32_t id): p_(x, y, z), v_(0,0,0), id_(id) {}


void LED::setState(bool state, size_t currentFrameTime) {

    state_ = state;


    if (state_) {
        numberOnBits_++;
    } else {

        if (numberOnBits_ >= 12) { //Check for start signal. Long on time followed by off.
            currentBit_ = 0;
            code_ = 0;
        }

        numberOnBits_ = 0;

    }

    if (currentBit_ > 0) {
	
	if (currentBit_ < 9) code_ |= (0x01&state_)<<(8-currentBit_);
        else if (currentBit_ < 11) code_ |= (0x01&state_)<<(currentBit_-1);
        else {
            
            //Check parity
            uint8_t numBitsOdd = 0;
            uint8_t numBitsEven = 0;
            for (size_t i = 0; i < 10; i++) {
                if ((0x01<<(7-i))&code_) {
                    if (i%2 == 0) numBitsEven++;
                    else numBitsOdd++;
                }
            }

            //Finished receiving coding. Get id and check if correct.
            currentBit_ = -1;

            if (numBitsOdd%2 == !(0x01&(code_>>8)) && numBitsEven%2 == !(0x01&(code_>>9))) id_ = code_&0xFF; //
	    //id_ = code_&0xFF;
        }

    }
    if (currentBit_ >= 0) currentBit_++;



    if (state_ && !lastState_) {

        lastStateTime_ = currentFrameTime;
        lastState_ = state_;

    } else if (!state_ && lastState_) {

        lastState_ = state_;

        int64_t dTime = currentFrameTime - lastStateTime_;
        lastStateTime_ = currentFrameTime;

        //if (wasOff_) id_ = dTime/2;

        wasOff_ = true;

    }

}

void LED::setPos(cv::Point3f point, bool updateTime, size_t currentFrameTime) {
    if (updateTime) lastPosSet_ = currentFrameTime;
    p_ = point;
}

void LED::setVel(cv::Point3f vel) {
    v_ = vel;
}

void LED::setId(int32_t id) {
    id_ = id;
}

int64_t LED::getLastStateChange() {
    return lastStateTime_;
}

int64_t LED::getLastPosChange() {
    return lastPosSet_;
}

int32_t LED::getId() const {
    return id_;
}

cv::Point3f LED::getPos() const {
    return p_;
}

cv::Point3f LED::getVel() const {
    return v_;
}

bool LED::getState() const {
    return state_;
}

LED LED::getPredictionAtTime(int64_t time) {

    LED pred(*this);
    pred.setPos(pred.getPos() + pred.getVel()*float(time - pred.getLastPosChange()));

    return pred;

}

void LED::drawLED(cv::Mat& image) const {

    std::ostringstream text;
    text << id_;

    cv::Scalar textColor(255, 0, 0);
    if (state_) textColor = cv::Scalar(0, 0, 255);

    cv::circle(image, cv::Point(p_.x, p_.y), 2, textColor, -1);
    //cv::line(image, cv::Point(p_.x, p_.y), cv::Point(p_.x, p_.y) + cv::Point(v_.x*2, v_.y*2), cv::Scalar(255,0,0), 2);
    cv::putText(image, text.str(), cv::Point(p_.x, p_.y) + cv::Point(10, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

}
