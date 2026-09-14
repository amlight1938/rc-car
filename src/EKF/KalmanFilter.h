#pragma once

#include "Arduino.h"
#include <ArduinoEigen.h>
#include "EKF/MeasurementState.h"

using namespace Eigen;

class KalmanFilter
{
public:
	KalmanFilter(double xStartPos, double yStartPos, double startAngle, double startRotationRate);

	void iterate(MeasurementState measurementState);

	double getX();
	double getY();
	double getTheta();
	double getOmega();

private:
	// creating the 4x1 X matrix and 1x2 Z Matrix
	Matrix<double, 1, 2> Z;
	Matrix<double, 4, 1> X;

	// creating the 2x2 R matrix and 2x4 H matrix
	Matrix<double, 2, 2> R;
	Matrix<double, 2, 4> H;

	// creating the Aj matrix, the jacobian of f(X)
	Matrix<double, 4, 4> Aj;

	// creating the P and Q matrices
	Matrix<double, 4, 4> P;
	Matrix<double, 4, 4> Q;

	void updateAj();

	double v;
	double prevTime;
	double dt;
};