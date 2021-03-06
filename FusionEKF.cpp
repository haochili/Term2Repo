#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <stdio.h>
#include <math.h>

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


  //measurement covariance matrix - laser
  R_laser_ << 2.25, 0,
        0, 2.25;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.9;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ <<1, 0, 0, 0,
        0, 1, 0, 0;

  F_ = MatrixXd (4, 4);
  F_<< 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

Q_ = MatrixXd(4, 4);
Q_<< 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
  //set the acceleration noise components

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
      float x_r = measurement_pack.raw_measurements_(0) * cos(measurement_pack.raw_measurements_(1));
      float y_r = measurement_pack.raw_measurements_(0) * sin(measurement_pack.raw_measurements_(1));
      float x_dot_r = measurement_pack.raw_measurements_(2) * cos(measurement_pack.raw_measurements_(1));
      float y_dot_r = measurement_pack.raw_measurements_(2) * sin(measurement_pack.raw_measurements_(1));

      ekf_.x_<<x_r, y_r, x_dot_r, y_dot_r;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ <<measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 1, 1;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.Init(ekf_.x_, P_, F_, H_laser_, R_radar_,  R_laser_, Q_);
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
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
   previous_timestamp_ = measurement_pack.timestamp_;
   float dt_2 = dt * dt;
   float dt_3 = dt_2 * dt;
   float dt_4 = dt_3 * dt;
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;

   int noise_ax = 9;
   int noise_ay = 9;
   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_  << 10, 0, 0, 0,
       0, 10, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;

   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ <<  dt_4/4.0*noise_ax, 0, dt_3/2.0*noise_ax, 0,
          0, dt_4/4.0*noise_ay, 0, dt_3/2.0*noise_ay,
          dt_3/2.0*noise_ax, 0, dt_2*noise_ax, 0,
          0, dt_3/2.0*noise_ay, 0, dt_2*noise_ay;
//predict
   ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Radar updates
    ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
    // Laser updatesk
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  cout << "Hj_ =" << ekf_.Hj_ << endl;

}
