#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double startPosX, double startPosY, double startAngle, double startRotationRate)
{
    // assuming velocity starts at 0
    v = 0;

    // assuming the time step is approx 10ms
    dt = 0.001;   // in units of seconds
    prevTime = 0; // in units of seconds

    // initializing the Z and X matrices
    Z << startAngle, startRotationRate;
    X << startPosX, startPosY, startAngle, startRotationRate;

    // initializing the R matrix
    // uncertainties are small due to radians being a small unit
    R << 0.001, 0,
        0, 0.001;
    H << 0, 0, 1, 0,
        0, 0, 0, 1;

    // initializing the Aj matrix
    Aj << 1, 0, -v * sin(X(2, 0)), 0,
        0, 1, v * cos(X(2, 0)), 0,
        0, 0, 1, dt,
        0, 0, 0, 1;

    // initializing the P and Q matrices
    // starting car at origin allow the x and y uncertainty to be very small
    P << 0.05, 0, 0, 0,
        0, 0.05, 0, 0,
        0, 0, R(0, 0), 0,
        0, 0, 0, R(1, 1);
    Q << 0.1, 0, 0, 0,
        0, 0.1, 0, 0,
        0, 0, 0.005, 0,
        0, 0, 0, 0.001;
}

void KalmanFilter::iterate(MeasurementState measurementState)
{
    // instantiating the matrices only used in this function
    Matrix<double, 4, 1> Xp;
    Matrix<double, 4, 4> Pp;
    Matrix<double, 4, 2> K;

    double currentTime = millis() / 1000.0; // using .0 to force floating point calculations
    dt = currentTime - prevTime;
    prevTime = currentTime;

    // computing the Xp matrix
    // Xp = f(Xprev) using previous state value of v and theta
    Xp(0, 0) = X(0, 0) + (v * dt * cos(X(2, 0)));
    Xp(1, 0) = X(1, 0) + (v * dt * sin(X(2, 0)));
    Xp(2, 0) = X(2, 0) + (dt * X(3, 0));
    Xp(3, 0) = X(3, 0);

    // updating the Aj matrix before computing Pp
    updateAj();

    // computing the Pp matrix
    // Pp = Aj*P*(Aj^T) + Q
    Pp = (Aj * P * Aj.transpose()) + Q;

    // computing the Kalman Gain matrix
    // K = Pp*(H^T)*(H*Pp*(H^T)+R)^(-1)
    K = Pp * H.transpose() * ((H * Pp * H.transpose()) + R).inverse();

    // gathering new data
    v = measurementState.velocity;
    Z << measurementState.theta, measurementState.omega;

    // computing the new X and P matrices
    // X = Xp + K*(Z-H*Xp)
    X = Xp + (K * (Z - (H * Xp)));
    // P = Pp - K*H*Pp
    P = Pp - (K * H * Pp);
}

double KalmanFilter::getX()
{
    return X(0, 0);
}

double KalmanFilter::getY()
{
    return X(1, 0);
}

double KalmanFilter::getTheta()
{
    return X(2, 0);
}

double KalmanFilter::getOmega()
{
    return X(3, 0);
}

void KalmanFilter::updateAj()
{
    Aj(0, 2) = -v * sin(X(2, 0));
    Aj(1, 2) = v * cos(X(2, 0));
    Aj(2, 3) = dt;
}