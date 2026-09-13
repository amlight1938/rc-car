#pragma once

#include "Arduino.h"
#include <ArduinoEigen.h>

using namespace Eigen;

class KalmanFilter
{
public:
    KalmanFilter(double xStartPos, double yStartPos, double startAngle, double startRotation, long startTime);

    void iterate(double theta, double w, double velocity, long currentTime);

    double getX();
    double getY();
    double getTheta();
    double getW();

private:
    // creating the 4x1 X matrix and 1x2 Z Matrix
    Matrix<double, 1, 2> Z;
    Matrix<double, 4, 1> X;

    // creating the 2x2 R matrix and 2x4 H matrix
    Matrix<double, 2, 2> R;
    Matrix<double, 2, 4> H;

    // creating the A matrix, the jacobian of f(X)
    Matrix<double, 4, 4> A;

    // creating the P and Q matrices
    Matrix<double, 4, 4> P;
    Matrix<double, 4, 4> Q;

    // creating the predicted matrices
    Matrix<double, 4, 1> Xp;
    Matrix<double, 4, 4> Pp;

    // creating the Kalman Gain matrix
    Matrix<double, 4, 2> K;

    double x;
    double y;
    double theta;
    double w;

    double v;
    double prevTime;
    double dt;
};