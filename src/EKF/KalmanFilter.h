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
	KalmanFilter(double xStartPos = 0, double yStartPos = 0, double startVel = 0, double startAccel = 0, double startAngle = 0, double startRotationRate = 0);

	void iterate(const MeasurementState &measurementState);

	Vector<double, 6> getState();

private:
	// creating the 6-dim X vector and 4-dim Z vector
	Vector<double, 4> Z;
	Vector<double, 6> X;

	// creating the 4x4 R matrix and 2x4 H matrix
	Matrix<double, 4, 4> R;
	Matrix<double, 4, 6> H;

	// creating the Aj matrix, the jacobian of f(X)
	Matrix<double, 6, 6> Aj;

	// creating the P and Q matrices
	Matrix<double, 6, 6> P;
	Matrix<double, 6, 6> Q;

	void updateAj();

	bool validateMeasurementState(const MeasurementState& state);

	double dtOffset = 0;
	double dt;
};