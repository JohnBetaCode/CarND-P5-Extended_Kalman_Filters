#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

// =============================================================================
/* 
In FusionEKF.cpp, we have given some starter code for implementing sensor 
fusion. In this file, you won't need to include the actual Kalman filter 
equations; instead, you will be initializing variables, initializing the 
Kalman filters, and then calling functions that implement the prediction 
step or update step. You will see TODO comments indicating where to put 
your code.

You will need to:
  1. (Done) initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
  2. (Done) initialize the Kalman filter position vector with the first sensor 
     measurements
  3. (Done) modify the F and Q matrices prior to the prediction step based on the 
     elapsed time between measurements
  4. (Done) call the update step for either the lidar or radar sensor measurement. 
     Because the update step for lidar and radar are slightly different, there 
     are different functions for updating lidar and radar.
*/

// =============================================================================
// CONSTRUCTOR - DESTRUCTOR - CONSTRUCTOR - DESTRUCTOR - CONSTRUCTOR - DESTRUCTO
// =============================================================================
FusionEKF::FusionEKF() {

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09,   0,        0,
              0,      0.0009,   0,
              0,      0,        0.09;

  /**
  * TODO: (Done)
  * initializing the FusionEKF. Setting the process 
  * and measurement noises
  */

  // P_in Initial state covariance
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1,   0,  0,    0,
             0,   1,  0,    0,
             0,   0,  1000, 0,
             0,   0,  0,    1000;

  // H_in Measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

}

FusionEKF::~FusionEKF() {}

// =============================================================================
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {

    /** TODO: (Done)
    * Initialize the state ekf_.x_ with the first measurement.
    * Create the covariance matrix. You'll need to convert 
    * radar from polar to cartesian coordinates.
    */

    // first measurement
    // cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "EKF - 1st measurement from RADAR" << endl;

      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = measurement_pack.raw_measurements_[0]; // range
  	  double phi = measurement_pack.raw_measurements_[1]; // bearing
  	  double rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
  	  
      // Coordinates convertion from polar to cartesian
  	  double x = rho * cos(phi);
      if ( x < 0.0001 ) {
        x = 0.0001;
      }
      double y = rho * sin(phi);
      if ( y < 0.0001 ) {
        y = 0.0001;
      }
      // Get velocities
      double vx = rho_dot * cos(phi);
  	  double vy = rho_dot * sin(phi);
      ekf_.x_ << x, y, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO:(Done) Initialize state.
      // No velocity and coordinates are cartesian already.
      cout << "EKF - 1st measurement from LASER" << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Saving the first timestamp in seconds
    previous_timestamp_ = measurement_pack.timestamp_ ;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // =============================================================================
  // PREDICTION STEP - PREDICTION STEP - PREDICTION STEP - PREDICTION STEP - PREDI
  // =============================================================================

  /**
  * TODO: (Done) Update the state transition matrix F according to the new elapsed time.
  * Time is measured in seconds.
  * TODO: (Done) Update the process noise covariance matrix.
  * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // State transition matrix update
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1,   0,  dt,  0,
             0,   1,  0,   dt,
             0,   0,  1,   0,
             0,   0,  0,   1;

  // Noise covariance matrix computation
  // Noise values from the task
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  double dt_2 = dt * dt;    //dt^2
  double dt_3 = dt_2 * dt;  //dt^3
  double dt_4 = dt_3 * dt;  //dt^4
  double dt_4_4 = dt_4 / 4; //dt^4/4
  double dt_3_2 = dt_3 / 2; //dt^3/2
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
              0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
              dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  // =============================================================================
  // UPDATE STEP - UPDATE STEP - UPDATE STEP - UPDATE STEP - UPDATE STEP - UPDATE 
  // =============================================================================
  /**
  * TODO: (Done)
  * - Use the sensor type to perform the update step.
  * - Update the state and covariance matrices.
  */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: (Done) Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: (Done) Laser updates
    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
