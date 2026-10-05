#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double startPosX, double startPosY, double startVel, double startAccel, double startAngle, double startRotationRate)
{
	// initializing the Z and X matrices
	X << startPosX, startPosY, startVel, startAccel, startAngle, startRotationRate;

	// initializing the R matrix
	// uncertainties are small due to radians being a small unit
	// values were derived from variances_calculation.py using stationary and relative movement data
	R << 100, 0, 0, 0,
		0, 100, 0, 0,
		0, 0, 0.0337665, 0,   // Tuned for gradual movements (see Encoder.h for other R values)
		0, 0, 0, 8.035852e-07;

	// both Omega values correspond to X[5]
	H << 0, 0, 1, 0, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1,
		0, 0, 0, 0, 0, 1;

	// assuming the time step is approx 10ms
	dt = 0.01; // seconds

	// initializing the Aj matrix
	updateAj();

	// initializing the P and Q matrices
	// starting car at origin allow the x and y uncertainty to be very small
	P << 0.0001, 0, 0, 0, 0, 0, // 1 cm uncertainty
		0, 0.0001, 0, 0, 0, 0,	// 1 cm uncertainty
		0, 0, 100, 0, 0, 0, 0,	// 100 cm/s uncertainty
		0, 0, 0, 100, 0, 0,		// 100 cm/s uncertainty
		0, 0, 0, 0, 0.001, 0,	// 2 degrees/s uncertainty
		0, 0, 0, 0, 0, 0.001;	// 2 degrees/s uncertainty
	Q << 1e-5, 0, 0, 0, 0, 0, // 1 cm
		0, 1e-5, 0, 0, 0, 0,	// 1 cm
		0, 0, 100, 0, 0, 0, 0,	// 100 cm/s
		0, 0, 0, 100, 0, 0,		// 100 cm/s
		0, 0, 0, 0, 1e-4, 0,	// - this is the size of the noise added each step
		0, 0, 0, 0, 0, 0.0015;
}

void KalmanFilter::iterate(const MeasurementState &measurementState)
{
	// instantiating the matrices only used in this function
	Vector<double, 6> Xp;
	Matrix<double, 6, 6> Pp;
	Matrix<double, 6, 4> K;

	dt = measurementState.timeSinceLastMeasurement; // seconds

	// computing the Xp matrix
	// Xp = f(Xprev) using previous state value of v
	Xp[0] = X[0] + (X[2] * dt * cos(X[4])) + (X[3] * (dt * dt / 2) * cos(X[4]));
	Xp[1] = X[1] + (X[2] * dt * sin(X[4])) + (X[3] * (dt * dt / 2) * sin(X[4]));
	Xp[2] = X[2] + (dt * X[3]);
	Xp[3] = X[3];
	Xp[4] = X[4] + (dt * X[5]);
	Xp[5] = X[5];

	// updating the Aj matrix before computing Pp
	updateAj();

	// computing the Pp matrix
	Pp = (Aj * P * Aj.transpose()) + Q;

	// computing the Kalman Gain matrix
	K = Pp * H.transpose() * ((H * Pp * H.transpose()) + R).inverse();

	// gathering new data
	Z << measurementState.velocity, measurementState.acceleration, measurementState.omega_encoder, measurementState.omega_gyro;

	// saving the innovation for troubleshooting and tuning of the EKF
	Vector<double, 4> innovation = Z - (H * Xp); 
	Vector<double, 4> normalized_innovation; //used for plotting/tuning filter. This compares the actual residual and the expected residual (expected comes from what Pp and R currently are)
	normalized_innovation[0] = innovation[0] / std::sqrt(Pp(3,3) + R(0,0)); //encoder
	normalized_innovation[1] = innovation[1] / std::sqrt(Pp(3,3) + R(1,1)); //gyro

	// computing the new X matrix
	X = Xp + (K * innovation);

	// making sure theta is between 0 and 2π
	X[4] = std::fmod(X[4], 2 * PI);
	if (X[4] < 0)
	{
		X[4] += 2 * PI;
	}

	// computing the new P matrix
	P = Pp - (K * H * Pp);

	// logging EKF values for troubleshooting and tuning of the EKF
	//NEED UPDATING DUE TO NEW MATRIX DIMENSIONS
	Logger::getInstance().log(true, "%6.1f- nrE: %-7.4f nrG: %-7.4f x: %-7.4f y: %-7.4f o: %-7.4f w: %-7.4f",
							  millis() / 1000.0, normalized_innovation[0], normalized_innovation[1], X[0], X[1], X[2], X[3]);
	Logger::getInstance().log(true, "%6.1f- Px: %.3e Py: %.3e Po: %.3e Pw: %.3e",
						 	  millis() / 1000.0, P(0, 0), P(1, 1), P(2, 2), P(3, 3));
}

Vector<double, 6> KalmanFilter::getState()
{
	return X;
}

void KalmanFilter::updateAj()
{
	Aj << 1, 0, dt * cos(X[4]), (dt * dt / 2) * cos(X[4]), (-X[2] * dt * sin(X[4])) + (-X[3] * (dt * dt / 2) * sin(X[4])), 0,
		0, 1, dt * sin(X[4]), (dt * dt / 2) * sin(X[4]), (X[2] * dt * cos(X[4])) + (X[3] * (dt * dt / 2) * cos(X[4])), 0,
		0, 0, 1, dt, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, dt,
		0, 0, 0, 0, 0, 1;
}
