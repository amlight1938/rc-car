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
	// creating a constructor with default arguments
	KalmanFilter(double xStartPos = 0, double yStartPos = 0, double startAngle = 0, double startRotationRate = 0);

	void iterate(const MeasurementState &measurementState);

	Vector<double, 4> getState();

private:
	// creating the 4-dim X vector and 2-dim Z vector
	Vector<double, 2> Z;
	Vector<double, 4> X;

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