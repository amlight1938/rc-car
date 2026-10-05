#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double startPosX, double startPosY, double startAngle, double startRotationRate)
{
	// assuming velocity starts at 0
	v = 0;

	// initializing the Z and X matrices
	X << startPosX, startPosY, startAngle, startRotationRate;

	// initializing the R matrix
	// uncertainties are small due to radians being a small unit
	// values were derived from variances_calculation.py using stationary and relative movement data
	R << 4.0020766, 0,
		0, 8.035852e-07;

	// both Z values correspond to X[3]
	H << 0, 0, 0, 1,
		0, 0, 0, 1;

	// assuming the time step is approx 10ms
	dt = 0.01; // seconds

	// initializing the Aj matrix
	Aj << 1, 0, -v * dt * sin(X[2]), 0,
		0, 1, v * dt * cos(X[2]), 0,
		0, 0, 1, dt,
		0, 0, 0, 1;

	// initializing the P and Q matrices
	// starting car at origin allow the x and y uncertainty to be very small
	P << 0.0001, 0, 0, 0, // 1 cm uncertainty
		0, 0.0001, 0, 0,  // 1 cm uncertainty
		0, 0, 0.001, 0,	  // 2 degrees uncertainty
		0, 0, 0, 0.001;	  // 2 degrees uncertainty
	Q << 0.0001, 0, 0, 0, // 1 cm
		0, 0.0001, 0, 0,  // 1 cm
		0, 0, 0.007, 0,	  // 5 degrees - this is the size of the noise added each step
		0, 0, 0, 0.007;	  // 5 degrees
}

void KalmanFilter::iterate(const MeasurementState &measurementState)
{
	// instantiating the matrices only used in this function
	Vector<double, 4> Xp;
	Matrix<double, 4, 4> Pp;
	Matrix<double, 4, 2> K;

	dt = measurementState.timeSinceLastMeasurement; // seconds

	// computing the Xp matrix
	// Xp = f(Xprev) using previous state value of v
	Xp[0] = X[0] + (v * dt * cos(X[2]));
	Xp[1] = X[1] + (v * dt * sin(X[2]));
	Xp[2] = X[2] + (dt * X[3]);
	Xp[3] = X[3];

	// updating the Aj matrix before computing Pp
	updateAj();

	// computing the Pp matrix
	Pp = (Aj * P * Aj.transpose()) + Q;

	// computing the Kalman Gain matrix
	K = Pp * H.transpose() * ((H * Pp * H.transpose()) + R).inverse();

	// gathering new data
	v = measurementState.velocity;

	Z << measurementState.omega_encoder, measurementState.omega_gyro;

	// saving the innovation for troubleshooting and tuning of the EKF
	Vector<double,2> innovation = Z - (H * Xp); 
	Vector<double,2> normalized_innovation; //used for plotting/tuning filter. This compares the actual residual and the expected residual (expected comes from what Pp and R currently are)
	normalized_innovation[0] = innovation[0] / std::sqrt(Pp(3,3) + R(0,0)); //encoder
	normalized_innovation[1] = innovation[1] / std::sqrt(Pp(3,3) + R(1,1)); //gyro

	// computing the new X matrix
	X = Xp + (K * innovation);

	// making sure theta is between 0 and 2π
	X[2] = std::fmod(X[2], 2 * PI);
	if (X[2] < 0)
	{
		X[2] += 2 * PI;
	}

	// computing the new P matrix
	P = Pp - (K * H * Pp);

	// logging EKF values for troubleshooting and tuning of the EKF
	Logger::getInstance().log(true, "%6.1f- nrE: %-7.4f nrG: %-7.4f x: %-7.4f y: %-7.4f o: %-7.4f w: %-7.4f",
							  millis() / 1000.0, normalized_innovation[0], normalized_innovation[1], X[0], X[1], X[2], X[3]);
	Logger::getInstance().log(true, "%6.1f- Px: %.3e Py: %.3e Po: %.3e Pw: %.3e",
						 	  millis() / 1000.0, P(0, 0), P(1, 1), P(2, 2), P(3, 3));
}

Vector<double, 4> KalmanFilter::getState()
{
	return X;
}

void KalmanFilter::updateAj()
{
	Aj(0, 2) = -v * dt * sin(X[2]);
	Aj(1, 2) = v * dt * cos(X[2]);
	Aj(2, 3) = dt;
}