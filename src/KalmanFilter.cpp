#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double startPosX, double startPosY, double startAngle, double startRotation, long startTime)
{
    // assuming velocity starts at 0
    v = 0;

    // assuming the time step is approx 10ms
    dt = 0.001;
    prevTime = (double)startTime / 1000;

    // initializing the Z and X matrices
    Z << startAngle, startRotation;
    X << startPosX, startPosY, startAngle, startRotation;

    // initializing the R matrix
    // uncertainties are small due to radians being a small unit
    R << 0.001, 0,
        0, 0.001;
    H << 0, 0, 1, 0,
        0, 0, 0, 1;

    // initializing the A matrix
    A << 1, 0, -v * sin(theta), 0,
        0, 1, v * cos(theta), 0,
        0, 0, 1, dt,
        0, 0, 0, 1;

    // initializing the P and Q matrices
    // starting car at origin allow the x and y uncertainty to be very small
    P << 0.05, 0, 0, 0,
        0, 0.05, 0, 0,
        0, 0, 0.002, 0,
        0, 0, 0, 0.0001;
    Q << 0.1, 0, 0, 0,
        0, 0.1, 0, 0,
        0, 0, 0.005, 0,
        0, 0, 0, 0.001;
}

void KalmanFilter::iterate(double theta, double w, double velocity, long currentTime)
{
    // computing the Xp matrix
    // Xp = f(Xprev)
    Xp(0, 0) = X(0, 0) + (v * dt * cos(X(2, 0)));
    Xp(1, 0) = X(1, 0) + (v * dt * sin(X(2, 0)));
    Xp(2, 0) = X(2, 0) + (dt * X(3, 0));
    Xp(3, 0) = X(3, 0);

    // computing the Pp matrix
    // Pp = A*P*(A^T) + Q
    Pp = (A * P * A.transpose()) + Q;

    // computing the Kalman Gain matrix
    // K = Pp*(H^T)*(H*Pp*(H^T)+R)^(-1)
    K = Pp * H.transpose() * ((H * Pp * H.transpose()) + R).inverse();

    // gathering new data
    v = velocity;
    Z << theta, w;
    dt = ((double)currentTime / 1000) - prevTime;
    prevTime = currentTime;

    // computing the new X and P matrices
    // X = Xp + K*(Z-H*Xp)
    X = Xp + (K * (Z - (H * Xp)));
    // P = Pp - K*H*Pp
    P = Pp - (K * H * Pp);
}

double KalmanFilter::getX()
{
    return x;
}

double KalmanFilter::getY()
{
    return y;
}

double KalmanFilter::getTheta()
{
    return theta;
}

double KalmanFilter::getW()
{
    return w;
}