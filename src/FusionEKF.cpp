/* code walkthrough: https://www.youtube.com/watch?v=J7WK9gEUltM&feature=youtu.be */
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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO[check]:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // H_laser matrix (...from Lesson 5, Section 10)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // initial state transition matrix (4x4)
  ekf_.F_ = MatrixXd(4,4); 
  ekf_.F_ << 1, 0, 0, 0, //(...walkthrough 24:34) 
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  //  state covariance matrix (4x4)
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // set the acceleration noise components
  noise_ax = 9; //aka: (sigma_ax)^2. Value of '9' provided in quiz from Lesson 5, Section 13
  noise_ay = 9; //aka: (sigma_ay)^2. Value of '9' provided in quiz from Lesson 5, Section 13
  
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
    ekf_.x_ << 1, 1, 1, 1; // last 2 elements very important for RMSE

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho     = measurement_pack.raw_measurements_(0);
      float phi     = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
   
      ekf_.x_(0) = rho*cos(phi);// set ekf_.x_(0) to ro*cos(phi) (...23.41 in walkthrough video)
      ekf_.x_(1) = rho*sin(phi);// set ekf_.x_(1) to ro*sin(phi)

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float x = measurement_pack.raw_measurements_(0);
      float y = measurement_pack.raw_measurements_(1);

      ekf_.x_(0) = x;// just set ekf_.x_(0) to x
      ekf_.x_(1) = y;// just set ekf_.x_(1) to y
    }

    // set the previous timestamp
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
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;// dt units are seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt*dt;// dt^2
  float dt_3 = dt_2*dt;// dt^3
  float dt_4 = dt_3*dt;// dt^4

  // Modify the F matrix so that the time is integrated (Lesson 5, Section 9)
  ekf_F_(0,2) = dt;
  ekf_F_(1,3) = dt;

  // Set the process covariance matrix "Q" (Lesson 5, Section 10)
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << (dt_4/4)*noise_ax, 0, (dt_3/2)*noise_ax, 0,
             0, (dt_4/4)*noise_ay, 0, (dt_3/2)*noise_ay, 
             (dt_3/2)*noise_ax, 0, (dt_2)*noise_ax, 0,
             0, (dt_3/2)*noise_ay, 0, (dt_2)*noise_ay;              

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  /* --------------------- BOOKMARK ------------------*/


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
