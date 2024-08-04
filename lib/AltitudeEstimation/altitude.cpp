#include "filters.h"
#include "algebra.h"
#include "altitude.h"

AltitudeEstimator::AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro,
                                     float ca, float accelThreshold)
:kalman(ca, sigmaGyro, sigmaAccel), complementary(sigmaAccel, sigmaBaro, accelThreshold)
{
      this->sigmaAccel = sigmaAccel;
      this->sigmaGyro = sigmaGyro;
      this->sigmaBaro = sigmaBaro;
      this->ca = ca;
      this->accelThreshold = accelThreshold;
}

void AltitudeEstimator::estimate(float accel[3], float gyro[3], float baroHeight, uint32_t timestamp)
{
        float deltat = (float)(timestamp-previousTime)/1000000.0f;
        float verticalAccel = kalman.estimate(pastGyro,
                                              pastAccel,
                                              deltat);
        complementary.estimate(& estimatedVelocity,
                               & estimatedAltitude,
                               baroHeight,
                               pastAltitude,
                               pastVerticalVelocity,
                               pastVerticalAccel,
                               deltat);
        copyVector(pastGyro, gyro);
        copyVector(pastAccel, accel);
        pastAltitude = estimatedAltitude;
        pastVerticalVelocity = estimatedVelocity;
        pastVerticalAccel = verticalAccel;
        previousTime = timestamp;
}

float AltitudeEstimator::getAltitude()
{
        return estimatedAltitude;
}

float AltitudeEstimator::getVerticalVelocity()
{
        return estimatedVelocity;
}

float AltitudeEstimator::getVerticalAcceleration()
{
        return pastVerticalAccel;
}