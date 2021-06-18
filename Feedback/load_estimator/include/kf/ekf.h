//
// Created by kevin on 2020/8/14.
//

#ifndef UKF_EKF_H
#define UKF_EKF_H

#include "kf/common_include.h"

class EKF {
public:
    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_vision_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_encoder_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    MatrixXd H_laser_;

    MatrixXd R_vision_;

    MatrixXd R_encoder_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_ax_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_ay_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_az_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_a1_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_a2_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_vipx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_vipy_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_vipz_;

    ///* Radar measurement noise standard deviation radius in m
    double std_ent1_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_ent2_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;




    /**
     * Constructor
     */
    EKF();

    /**
     * Destructor
     */
    virtual ~EKF();

    void transition_function(double delta_t, MeasurementPackage mearPack);

    void ProcessMeasurement(MeasurementPackage meas_package);

    Eigen::Vector3d measurement_function(double theta1, double theta2, double l);
};


#endif //UKF_EKF_H
