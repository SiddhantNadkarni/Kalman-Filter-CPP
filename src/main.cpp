#include "KalmanFilter.h"








int main(int argc, char const *argv[])
{


	int n(4); //dim of state vector
	int m(1); //dim of control vector
	int k(1);// dim of measurement vector

	Eigen::MatrixXd A(n, n); //state transition matrix
	Eigen::MatrixXd B(n, m); //maps control vector to state space 
	Eigen::MatrixXd C(k, n); //maps state vector to observation/measurment space 
	Eigen::MatrixXd R(n, n); //process noise covariance matrix
	Eigen::MatrixXd Q(k, k); //measurement noise covariance matrix
	Eigen::MatrixXd P(n, n); //final covariance matrix of gaussian distribution of 
	                         //robot belief

	double dt = 0.1;
    
    //initializing state transition and mapping matrices
	A << 1, 0.1, 0, 0,
	     0, 1, 0.1, 0,
	     0, 0, 1, 0.1,
	     0, 0, 0, 1;
	B.setZero();
	C << 1, 0, 0, 0;

    //initializing covariance matrices
    R << 0.1, 0, 0, 0,
	     0, 0.1, 0, 0,
	     0, 0, 0.1, 0,
	     0, 0, 0, 0.1;
	Q << 1;
    P << 10, 0, 0, 0,
	     0, 10, 0, 0,
	     0, 0, 10, 0,
	     0, 0, 0, 10;

	//Initialize Executime time
	struct timespec start, stop; 
	double time;

	//start execution time measurement
    if( clock_gettime(CLOCK_REALTIME, &start) == -1) { perror("clock gettime");}


    //Initialize state vector 
    Eigen::MatrixXd x0(n,1);

    x0 << 5,
          1,
          0,
          0;
    std::cout << x0 << std::endl;

    //Initialize kalman Filter object
    KalmanFilter kf(A, B, C, R, Q, P);
    kf.init(x0);


    //Initialize t = 0, measurement and control vector
    double t(0);
    Eigen::MatrixXd z(k,1);
    Eigen::MatrixXd u(m,1);

    //Initial State at t = 0
	std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;

	//Kalman Filter Algorithm
	for (int i = 0; i < 100; ++i)
	{
	  	z << sin(0.1*t);
	  	u << 0;
	  	kf.predict(u);
	  	kf.calculateKalmanGain();
	  	kf.update(z);
	  	t+=dt;
	  	std::cout << "t = " << t << ", " << "z[" << i << "] = " << z.transpose()
	    << ", x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
	}


	//stop execution time
	if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) { perror("clock gettime");}		
	time = (stop.tv_sec - start.tv_sec)+ (double)(stop.tv_nsec - start.tv_nsec)/1e9;

	std::cout << "Execution time = " << time << std::endl;
	return 0;
}