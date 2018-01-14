#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_	= MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225,	0,
		  	  		0, 			0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 			0,
		  	  		0, 		0.0009, 0,
							0, 		0, 			0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // init measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
			  			0, 1, 0, 0;

  // init Kalman Filter
  //init state vector to no-zero position and zero spead
  VectorXd x = VectorXd(4);
  x << 1, 1, 0, 0;

  //state covariance matrix P
  MatrixXd P = MatrixXd(4, 4);
  P << 	1, 0, 0, 0,
  			0, 1, 0, 0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;

  //state transition matrix
  MatrixXd F = MatrixXd(4, 4);
  F <<	1, 0, 1, 0,
  			0, 1, 0, 1,
				0, 0, 1, 0,
				0, 0, 0, 1;

  //process covariance matrix Q to zero
  MatrixXd Q = MatrixXd(4, 4);
  Q <<	1, 0, 1, 0,
  			0, 1, 0, 1,
				1, 0, 1, 0,
				0, 1, 0, 1;

  // init EKF
  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	ekf_.x_ = tools.PolarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }

    // update timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the state transition matrix F according to the new elapsed time
  ekf_.F_ <<	1, 0, dt, 0,
							0, 1, 0, dt,
							0, 0, 1, 0,
							0, 0, 0, 1;

  //Update the process noise covariance matrix
  float	noise_ax = 9.0,
  			noise_ay = 9.0;

  float	dt_2 = dt * dt,
  			dt_3 = dt_2 * dt,
				dt_4 = dt_3 * dt;

  ekf_.Q_ <<	dt_4/4*noise_ax,	0,								dt_3/2*noise_ax, 	0,
							0, 								dt_4/4*noise_ay, 	0,								dt_3/2*noise_ay,
							dt_3/2*noise_ax,	0, 								dt_2*noise_ax, 		0,
							0, 								dt_3/2*noise_ay, 	0, 								dt_2*noise_ay;

  //call the Kalman Filter predict() function
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  //update the state
	VectorXd z = measurement_pack.raw_measurements_;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  	//calculate Jacobian
  	Hj_ = tools.CalculateJacobian(ekf_.x_);

  	// Radar setup
    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj_;

  	// Radar updates
  	ekf_.UpdateEKF(z);
  } else {
  	// Laser setup
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    // Laser updates
  	ekf_.Update(z);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
