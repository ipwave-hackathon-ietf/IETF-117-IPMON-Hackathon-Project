#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

//#include "Eigen/Dense"
#include <eigen3/Eigen/Dense>

class MeasurementPackage {
public:
  double timestamp_;

  Eigen::VectorXd raw_measurements_;
  Eigen::VectorXd stateMeasErr_; //BAM extensions
};

#endif /* MEASUREMENT_PACKAGE_H_ */
