#pragma once

#include "Arduino.h"
#include <ArduinoEigen.h>
#include "EKF/MeasurementState.h"
#include "Logging/Logger.h"
#include <cmath>

using namespace Eigen;

class KalmanFilter
{
public:
	KalmanFilter(double xStartPos = 0, double yStartPos = 0, double startAngle = 0, double startRotationRate = 0);

	void iterate(const MeasurementState& measurementState);

	Matrix<double, 4, 1> getState();

private:
	// creating the 4x1 X matrix and 1x2 Z Matrix
	Vector2d Z;
	Vector4d X;

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
	double dt;

};