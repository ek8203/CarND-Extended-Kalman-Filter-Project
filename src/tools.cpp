#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if(estimations.size() == 0){
		cout << "CalculateRMSE() - Error - estimation vector size should not be zero" << endl;
		return rmse;
	}

	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()){
		cout << "CalculateRMSE() - Error - estimation vector size should equal ground truth vector size" << endl;
		return rmse;
	}
	// ... your code here

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){
        // ... your code here
		//cout << estimations [i] - ground_truth[i] << endl;
		VectorXd res = estimations [i] - ground_truth[i];
		res = res.array() * res.array();
		rmse += res;
	}

	//calculate the mean
	// ... your code here
	rmse = rmse / estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE
	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px + py*py;
	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		c1 = 0.0001;
	}

	float c2 = sqrt(c1);
	if(fabs(c2) < 0.001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		c2 = 0.001;
	}

	float c3 = (c1*c2);

	if(fabs(c3) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		c3 = 0.0001;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  	-(py/c1), (px/c1), 0, 0,
				py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}

VectorXd Tools::CartesianToPolar(const VectorXd& x_cort)	{
	/**
	 * Convert Cartesian coordinates to polar:
	 * Input x_cort is 4x1 vector in Cartesian coordinates: px, py,
	 * Output is a 3x1 vector in polar coordinate: rho, theta, rho_dot
	 */
	VectorXd x_polar(3);
	x_polar << 0, 0, 0;

	float rho;
	float theta;
	float rho_dot;

	// Cartesian coordinates
	float	px = x_cort(0),
		  	py = x_cort(1),
				vx = x_cort(2),
				vy = x_cort(3);

	if (fabs(px) < 0.0001) {
		px = 0.0001;
	}

	// calculate distance (range) rho
	rho = sqrt(px*px + py*py);

	if (fabs(rho) < 0.0001) {
		cout << "Tools::CartesianToPolar() - Error - Devide by zero!" << endl;
		rho = 0.0001;
	}

	// calculate angle theta in range -pi < theta < pi
	theta = atan2(py, px);

	// calculate range rate ro_dot
	rho_dot = (px*vx + py*vy)/rho;

	// output vector
	x_polar << rho, theta, rho_dot;

	return x_polar;
}

VectorXd Tools::PolarToCartesian(const VectorXd& x_polar)	{

	VectorXd x_cort(4);

	float rho = x_polar(0);
	float theta = x_polar(1);
	float rho_dot = x_polar(2);

	float px = rho * cos(theta);
	float py = rho * sin(theta);
	float vx = rho_dot * cos(theta);
	float vy = rho_dot * sin(theta);

	x_cort << px, py, vx, vy;

	return x_cort;
}

