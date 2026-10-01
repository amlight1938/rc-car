#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double startPosX, double startPosY, double startAngle, double startRotationRate)
{
	// assuming velocity starts at 0
	v = 0;

	// initializing the Z and X matrices
	X << startPosX, startPosY, startAngle, startRotationRate;

	// initializing the R matrix
	// uncertainties are small due to radians being a small unit
	// values derived from variance calculation with variances_calculation.py
	R << 4.0020766, 0,
		0, 8.035852e-07;
	H << 0, 0, 0, 1,
		0, 0, 0, 1;

	// assuming the time step is approx 10ms
	dt = 0.01;	  // in units of seconds

	// initializing the Aj matrix
	Aj << 1, 0, -v * dt* sin(X(2, 0)), 0,
		0, 1, v * dt* cos(X(2, 0)), 0,
		0, 0, 1, dt,
		0, 0, 0, 1;

	// initializing the P and Q matrices
	// starting car at origin allow the x and y uncertainty to be very small
	P << 0.0001, 0, 0, 0, // 1cm uncertainty
		0, 0.0001, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.001;
	Q << 0.0001, 0, 0, 0,
		0, 0.0001, 0, 0,
		0, 0, 0.007, 0,
		0, 0, 0, 0.007;
}

void KalmanFilter::iterate(const MeasurementState& measurementState)
{
	// instantiating the matrices only used in this function
	Vector4d Xp;
	Matrix<double, 4, 4> Pp;
	Matrix<double, 4, 2> K;

	dt = measurementState.timeSinceLastMeasurement; //seconds

	// Logger::getInstance().log(true, "dt: %-8.4f", dt );
	// Logger::getInstance().log(true, "v: %-8.4f", v );

	// computing the Xp matrix
	// Xp = f(Xprev) using previous state value of v
	Xp[0] = X[0] + (v * dt * cos(X[2]));
	Xp[1] = X[1] + (v * dt * sin(X[2]));
	Xp[2] = X[2] + (dt * X[3]);
	Xp[3] = X[3];

	// Logger::getInstance().log(true, "x: %-8.4f y: %-8.4f theta: %-8.4f w: %-8.4f", Xp[0], Xp[1], Xp[2], Xp[3]);

	// updating the Aj matrix before computing Pp
	updateAj();

	// computing the Pp matrix
	Pp = (Aj * P * Aj.transpose()) + Q;

	// computing the Kalman Gain matrix
	K = Pp * H.transpose() * ((H * Pp * H.transpose()) + R).inverse();

	// gathering new data
	v = measurementState.velocity;
	// Logger::getInstance().log(true, "v: %-8.4f measured_v: %-8.4f", v, measurementState.velocity );

	Z << measurementState.omega_encoder, measurementState.omega_gyro;

	// Logger::getInstance().log(true, "we: %-8.4f wg: %-8.4f Zwe: %-8.4f Zwg: %-8.4f", 
	// 	measurementState.omega_encoder, measurementState.omega_gyro, Z[0], Z[1]);
	// Logger::getInstance().log(true, "    " );
	// Logger::getInstance().log(true, "    " );
	// Logger::getInstance().log(true, "    " );


	// computing the new X and P matrices
	X = Xp + (K * (Z - (H * Xp)));
	X[2] = std::fmod(X[2], 2 * PI); 
	if(X[2] < 0){
		X[2] += 2 * PI;
	}
	P = Pp - (K * H * Pp);
}

Matrix<double, 4, 1> KalmanFilter::getState()
{
	return X;
}

void KalmanFilter::updateAj()
{
	Aj(0, 2) = -v * dt * sin(X(2, 0));
	Aj(1, 2) = v * dt * cos(X(2, 0));
	Aj(2, 3) = dt;
}