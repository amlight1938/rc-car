#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
	initialize();
}

void KalmanFilter::iterate(const MeasurementState &measurementState)
{
	if (!validateMeasurementState(measurementState))
	{
		dtOffset += measurementState.timeSinceLastMeasurement;
		return;
	}

	// instantiating the matrices only used in this function
	Vector<double, 6> Xp;
	Matrix<double, 6, 6> Pp;
	Matrix<double, 6, 4> K;

	dt = measurementState.timeSinceLastMeasurement + dtOffset; // seconds
	dtOffset = 0.0;

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
	Vector<double, 4> normalized_innovation;								  // used for plotting/tuning filter. This compares the actual residual and the expected residual (expected comes from what Pp and R currently are)
	normalized_innovation[0] = innovation[0] / std::sqrt(Pp(2, 2) + R(0, 0)); // velocity
	normalized_innovation[1] = innovation[1] / std::sqrt(Pp(3, 3) + R(1, 1)); // acceleration
	normalized_innovation[2] = innovation[2] / std::sqrt(Pp(5, 5) + R(2, 2)); // omega_encoder
	normalized_innovation[3] = innovation[3] / std::sqrt(Pp(5, 5) + R(3, 3)); // omega_gyro

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
	// Logger::getInstance().log(true, "%6.1f- nV: %-7.4f nA: %-7.4f nE: %-7.4f nG: %-7.4f Px: %.3e Py: %.3e Pv: %.3e Pa: %.3e Po: %.3e Pw: %.3e",
	// 						  millis() / 1000.0, normalized_innovation[0], normalized_innovation[1], normalized_innovation[2], normalized_innovation[3], P(0, 0), P(1, 1), P(2, 2), P(3, 3), P(4, 4), P(5, 5));
	// Logger::getInstance().log(true, "%6.1f- x: %-7.4f y: %-7.4f v: %-7.4f a: %-7.4f o: %-7.4f w: %-7.4f",
	// 						  millis() / 1000.0, X[0], X[1], X[2], X[3], X[4], X[5]);
	// Logger::getInstance().log(true, "%6.1f- Px: %.3e Py: %.3e Pv: %.3e Pa: %.3e Po: %.3e Pw: %.3e",
	// 						  millis() / 1000.0, P(0, 0), P(1, 1), P(2, 2), P(3, 3), P(4, 4), P(5, 5));
}

Vector<double, 7> KalmanFilter::getState()
{
	Vector<double, 7> fullState;
	fullState << X, dt;
	return fullState; // x, y, v, a, theta, omega, dt
}

void KalmanFilter::updateAj()
{
	Aj(0, 2) = dt * cos(X[4]);
	Aj(0, 3) = (dt * dt / 2) * cos(X[4]);
	Aj(0, 4) = (-X[2] * dt * sin(X[4])) + (-X[3] * (dt * dt / 2) * sin(X[4]));
	Aj(1, 2) = dt * sin(X[4]);
	Aj(1, 3) = (dt * dt / 2) * sin(X[4]);
	Aj(1, 4) = (X[2] * dt * cos(X[4])) + (X[3] * (dt * dt / 2) * cos(X[4]));
	Aj(2, 3) = dt;
	Aj(4, 5) = dt;
}

bool KalmanFilter::validateMeasurementState(const MeasurementState &state)
{
	return std::abs(state.velocity) < 1.0 &&
		   std::abs(state.acceleration) < 7.0 &&
		   std::abs(state.omega_encoder) < 10.0 &&
		   std::abs(state.omega_gyro) < 6.0;
}

void KalmanFilter::reset(){
	dtOffset = 0;
	dt = 0.01; //set back to the assumed 10ms
	initialize();
}

void KalmanFilter::initialize(){
	// initializing the X matrices
	X << 0, 0, 0, 0, 0, 0;

	// initializing the Aj matrix
	Aj << 1, 0, dt * cos(X[4]), (dt * dt / 2) * cos(X[4]), (-X[2] * dt * sin(X[4])) + (-X[3] * (dt * dt / 2) * sin(X[4])), 0,
		0, 1, dt * sin(X[4]), (dt * dt / 2) * sin(X[4]), (X[2] * dt * cos(X[4])) + (X[3] * (dt * dt / 2) * cos(X[4])), 0,
		0, 0, 1, dt, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, dt,
		0, 0, 0, 0, 0, 1;

	// initializing the P and Q matrices
	// starting car at origin allow the x and y uncertainty to be very small
	P << 0.0001, 0, 0, 0, 0, 0, // 1 cm uncertainty
		0, 0.0001, 0, 0, 0, 0,	// 1 cm uncertainty
		0, 0, 0.0001, 0, 0, 0,	// 1 cm/s uncertainty
		0, 0, 0, 0.0001, 0, 0,	// 1 cm/s^2 uncertainty
		0, 0, 0, 0, 0.001, 0,	// 2 degrees/s uncertainty
		0, 0, 0, 0, 0, 0.001;	// 2 degrees/s uncertainty
}
