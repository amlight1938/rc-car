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
	KalmanFilter();

	void iterate(const MeasurementState &measurementState);

	Vector<double, 7> getState();

	void reset();

private:
	// creating the 6-dim X vector and 4-dim Z vector
	Vector<double, 4> Z;
	Vector<double, 6> X; // x, y, v, a, theta, omega

	// creating the Aj matrix, the jacobian of f(X)
	Matrix<double, 6, 6> Aj;

	Matrix<double, 6, 6> P;

	// initializing the R matrix
	// uncertainties are small due to radians being a small unit
	// values were derived from variances_calculation.py using stationary and relative movement data
	const Matrix<double, 4, 4> R{
		{0.5e-5, 0, 0, 0},		  // v, educated guess
		{0, 2.81e-4, 0, 0},		  // a, based on variance_calulation.py
		{0, 0, 0.0337, 0},		  // w_encoder, Tuned for gradual movements (see Encoder.h for other R values)
		{0, 0, 0, 8.035852e-07}}; // w_gyro, based on variance_calulation.py

	const Matrix<double, 4, 6> H{
		{0, 0, 1, 0, 0, 0},
		{0, 0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0, 1},
		{0, 0, 0, 0, 0, 1}}; // both Omega values correspond to X[5]

	const Matrix<double, 6, 6> Q{
		{1e-5, 0, 0, 0, 0, 0},	  // x, 1 cm
		{0, 1e-5, 0, 0, 0, 0},	  // y, 1 cm
		{0, 0, 1e-5, 0, 0, 0},	  // v, educated guess
		{0, 0, 0, 1e-1, 0, 0},	  // a, educated guess
		{0, 0, 0, 0, 1e-4, 0},	  // theta, educated guess on normalized innovation plots
		{0, 0, 0, 0, 0, 0.0015}}; // omega, educated guess on normalized innovation plots

	void initialize();

	void updateAj();

	bool validateMeasurementState(const MeasurementState &state);

	double dtOffset = 0;
	double dt = 0.01; // assuming the time step is approx 10ms
};