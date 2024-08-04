# pragma once

#include "filters.h"
#include "algebra.h"

#include <Arduino.h>

class AltitudeEstimator {

  private:
    float sigmaAccel;
    float sigmaGyro;
    float sigmaBaro;
    float ca;
    float accelThreshold;
    float g = 9.81;
    uint32_t previousTime = micros();
    KalmanFilter kalman;
    ComplementaryFilter complementary;
    float pastVerticalAccel = 0;
    float pastVerticalVelocity = 0;
    float pastAltitude = 0;
    float pastGyro[3] = {0, 0, 0};
    float pastAccel[3] = {0, 0, 0};
    float estimatedAltitude = 0;
    float estimatedVelocity = 0;

  public:

    AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro,
                      float ca, float accelThreshold);

    void estimate(float accel[3], float gyro[3], float baroHeight, uint32_t timestamp);

    float getAltitude();

    float getVerticalVelocity();

    float getVerticalAcceleration();

};