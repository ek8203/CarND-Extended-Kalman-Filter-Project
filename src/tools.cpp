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
	rmse = rmse.array() / estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = sqrt(rmse.array());

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   /**
    TODO:
    * Calculate a Jacobian here.
   */
	MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	float p1 = px*px + py*py;
	if (p1 == 0.0){
		cout << "Tools::CalculateJacobian() - Error - Devide by zero!" << endl;
		return Hj;
	}

	float p2 = sqrt(p1);
	float p3 = pow(p1, 1.5);

	//compute the Jacobian matrix
	Hj(0,0) = px/p2;
	Hj(0,1) = py/p2;
	Hj(1,0) = -py/p1;
	Hj(1,1) = px/p1;
	Hj(2,0) = py*(vx*py - vy*px)/p3;
	Hj(2,1) = px*(vy*px - vx*py)/p3;
	Hj(2,2) = px/p2;
	Hj(2,3) = py/p2;

	return Hj;
}
