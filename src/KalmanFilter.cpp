#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(Eigen::MatrixXd A, 
		Eigen::MatrixXd B, 
		Eigen::MatrixXd C, 
		Eigen::MatrixXd R, 
		Eigen::MatrixXd Q, 
		Eigen::MatrixXd P): A(A), B(B), C(C), Q(Q), R(R), P0(P), 
        n(A.rows()), m(B.cols()), k(C.rows()), I(n, n), x_hat(n,1)
{
	I.setIdentity();
	std::cout << "Kalman Filter created" << std::endl;
}

void KalmanFilter::init()
{
	x_hat.setZero();
	P = P0;
	iniFlag = true;
}

void KalmanFilter::init(Eigen::MatrixXd x0)
{
	x_hat = x0;
	P = P0;
	iniFlag = true;
}

void KalmanFilter::predict(Eigen::MatrixXd u)
{
	if(!iniFlag)
	{
		std::cout << "Kalman Filter not initialized" << std::endl;
	}

	x_hat = A*x_hat + B*u;
	P = A*P*A.transpose() + R;

}

void KalmanFilter::calculateKalmanGain()
{
	K = P*C.transpose()*(C*P*C.transpose() + Q).inverse();
}

void KalmanFilter::update(Eigen::MatrixXd z)
{
	x_hat += K*(z - C*x_hat);
	P = (I - K*C)*P;
}

Eigen::MatrixXd KalmanFilter::state()
{
	return x_hat;
}





KalmanFilter::~KalmanFilter()
{
	std::cout << "Kalman Filter destroyed" << std::endl;
}


