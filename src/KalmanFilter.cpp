#include "KalmanFilter.h"
#include "Arduino.h"

KalmanFilter::KalmanFilter(){}

void KalmanFilter::iterate(double theta)
{


}

double KalmanFilter::getX()
{
    return X;
}

double KalmanFilter::getY()
{
    return Y;
}

double KalmanFilter::getTheta()
{
    return _theta;
}