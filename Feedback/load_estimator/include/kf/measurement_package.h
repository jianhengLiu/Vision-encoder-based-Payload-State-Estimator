#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "eigen3/Eigen/Dense"

/**
 * @brief  传感器种类包括两种 encoder和vision
 *
 */


class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    ENCODER,
    VISION,
LASER,
RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
