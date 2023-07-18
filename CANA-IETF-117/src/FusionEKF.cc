#include "FusionEKF.h"
#include "tools.h"
#include <eigen3/Eigen/Dense>

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
  R_ = MatrixXd(3, 3);

  //measurement covariance matrix
  R_ << 0.09,    0.0,  0.0,
               0.0, 0.0009,  0.0,
               0.0,    0.0, 0.09;

  // measurement noises
  noise_ax_ = 1.5;
  noise_ay_ = 1.5;
  noise_az_ = 1.5;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        // initialization code

        cout << "the first measurement is received; "
                << "measurement is performed At "
                << "time " << measurement_pack.timestamp_
                << endl;

        // initialize x---the first measurement
        VectorXd x = VectorXd(6);

        // initial state in case the first measurement comes from lidar sensor
        x << measurement_pack.raw_measurements_[0],
             measurement_pack.raw_measurements_[1],
             measurement_pack.raw_measurements_[2],
                                               0.0,    // we have no info about the velocity for lidar measurement
                                               0.0,
                                               0.0;

        // state covariance matrix (initial velocity is unknown, hence the level of uncertainty is high)
        MatrixXd P(6, 6);
        P << 1, 0,0,    0,    0,    0,
             0, 1,0,    0,    0,    0,
             0, 0,1,    0,    0,    0,
             0, 0,0, 1000,    0,    0,
             0, 0,0,    0, 1000,    0,
             0, 0,0,    0,    0, 1000;

        // state transition matrix (initially Δt is 0)
        MatrixXd F(6, 6);
        F << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        // process covariance matrix (initially Δt is 0, hence Q consists of 0's;
        //                            Eigen initializes matrices with 0's by default)
        MatrixXd Q(6, 6);

        ekf_.Init(x, P, F,R_, Q);

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    //compute the time elapsed between the current and previous measurements
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    double dt_2 = dt   * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    bool flagignore = true;
    if (flagignore){
        // modify the state transition matrix F so that the time is integrated
        ekf_.F_(0, 2) = dt;
        ekf_.F_(1, 3) = dt;

        // update the process covariance matrix
        ekf_.Q_ = MatrixXd(6, 6);
        ekf_.Q_ <<  dt_4/4*noise_ax_,        0.0,    0.0,  dt_3/2*noise_ax_,         0.0,  0.0,
                               0.0, dt_4/4*noise_ay_, 0.0,       0.0,    dt_3/2*noise_ay_, 0.0,
                               0.0,  0.0, dt_4/4*noise_az_,      0.0,    0.0, dt_3/2*noise_az_,
                               dt_3/2*noise_ax_,    0.0, 0.0,    dt_2*noise_ax_,       0.0,0.0,
                               0.0, dt_3/2*noise_ay_, 0.0,       0.0,   dt_2*noise_ay_,    0.0,
                               0.0, 0.0, dt_3/2*noise_az_,       0.0,    0.0 ,  dt_2*noise_az_;

    }

    // prediction step
    ekf_.Predict();

    // update step
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    std::cout <<"Raw data: " <<measurement_pack.raw_measurements_ <<std::endl;
    // print the output
    std::cout << "==========" << std::endl
            << "x_ = "      << std::endl << ekf_.x_ << std::endl
            << "-----"      << std::endl
            << "P_ = "      << std::endl << ekf_.P_ << std::endl
            << "==========" << std::endl            << std::endl;
}
