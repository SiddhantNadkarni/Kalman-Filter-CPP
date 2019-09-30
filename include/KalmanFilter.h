#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <time.h>

class KalmanFilter
{
private:
	int n, m, k;
	Eigen::MatrixXd A, B, C, Q, R, P, P0, I, K, x_hat, P_hat, x;
	bool iniFlag = false;

public:
	KalmanFilter(Eigen::MatrixXd A, 
		Eigen::MatrixXd B, 
		Eigen::MatrixXd C, 
		Eigen::MatrixXd R, 
		Eigen::MatrixXd Q, 
		Eigen::MatrixXd P);
	void init();
	void init(Eigen::MatrixXd x0);
	void predict(Eigen::MatrixXd u);
	void calculateKalmanGain();
	void update(Eigen::MatrixXd z);

	Eigen::MatrixXd state();


	~KalmanFilter();
};