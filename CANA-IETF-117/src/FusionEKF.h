#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include "EKF.h"
#include "tools.h"

class FusionEKF {
    public:
        /**
         * * Constructor
         * */
        FusionEKF();

        /**
         * Destructor.
         */
        virtual ~FusionEKF();

        /**
         * Run the whole flow of the Kalman Filter from here.
         */
        void ProcessMeasurement(const MeasurementPackage &measurement_pack);

        /**
         * Kalman Filter update and prediction math lives in here.
         */
        KalmanFilter ekf_;

    private:
      // check whether the tracking toolbox was initialized or not (first measurement)
        bool is_initialized_ = false;

        // previous timestamp
        long long previous_timestamp_;

        // tool object used to compute Jacobian and RMSE
        Tools tools;
        Eigen::MatrixXd R_;

        // noises
        double noise_ax_;
        double noise_ay_;
        double noise_az_;
};
#endif /* FusionEKF_H_ */
