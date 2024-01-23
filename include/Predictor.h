#ifndef PREDICTOR_H
#define PREDICTOR_H


#include "stdint.h"

#include "LedClass.h"

#include "SatelliteClass.h"

#include <opencv2/opencv.hpp>


/**
 * @brief Used to predict LED positions, Satellite Pose etc.
 * 
 */
class Predictor {
public:

    /**
     * @brief Predict position of all LEDs with unconfirmed ID using their current velocity at the given time
     * 
     * @param leds 
     * @param time absolute time to predict LED position
     */
    void predictNoIDLEDPositions(std::vector<LED>& leds, long long time);

    /**
     * @brief Predicts the satellite pose at given time.
     * 
     * @param sat 
     * @param time 
     */
    void predictSatPose(Satellite& sat, long long time);

    /**
     * @brief Updates position of all LEDs with confirmed ID with the projection of the satellite model leds onto the display.
     * 
     * @param leds 
     * @param sat 
     */
    void predictSatLEDPositions(std::vector<LED>& leds, const Satellite& sat);


};





#endif