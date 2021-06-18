//
// Created by kevin on 2020/8/14.
//

#include "kf/ekf.h"
#include "kf/common_include.h"
//
// Created by abang on 17-10-26.
//
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


Eigen::MatrixXd jacob(double theta1, double theta2, double l){
    double deltat;


    double tan1 = tan(theta1);
    double tan2 = tan(theta2);
    double tan11 = tan1 * tan1;
    double tan22 = tan2 * tan2;
    double tan12 = tan1 * tan2;
    double l2 = l*l;
    double A = (tan11+ tan22 + 1);
    double B = sqrt((l2)/A);
    double C = B*A*A;


    double J11 = B * (tan11 + 1) - (l2*tan11)*(tan11 + 1)/C;
    double J12 = - l2*tan12*(tan22 + 1) / C;
    double J21 = - l2*tan12*(tan11 + 1) / C;
    double J22 = B*(tan2*tan2 + 1) - l2*tan22 *(tan22+1)/C;
    double J31 = - l2*tan1*(tan11 + 1)/C;
    double J32 = - l2*tan2*(tan22 + 1)/C;
    double J13 = l*tan1/A/B;
    double J23 = l*tan2/A/B;
    double J33 = l/A/B;

    Eigen::MatrixXd jaco(3,5);
    jaco<<J11, J12, J13, 0,0,
            J21, J22, J23,0,0,
            J31, J32, J33,0,0;

    return jaco;
}

Eigen::Vector3d EKF::measurement_function(double theta1, double theta2, double l){
    double z = sqrt(l*l/(1+tan(theta1) * tan(theta1)+ tan(theta2)*tan(theta2)));
    double x = z * tan(theta1);
    double y = z * tan(theta2);
    Eigen::Vector3d mear;
    mear<< x,y,z;
    return mear;
}


/**
 * Initializes Unscented Kalman filter
 */
EKF::EKF() {
    // if this is false, vision measurements will be ignored (except during init)
    use_vision_ = true;

    // if this is false, encoder measurements will be ignored (except during init)
    use_encoder_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a1_ = 0.1;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a2_ = 0.1;

    // Laser measurement noise standard deviation position1 in m
    std_ent1_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_ent2_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_vipx_ = 0.005;

    // Radar measurement noise standard deviation angle in rad
    std_vipy_ = 0.005;

    // Radar measurement noise standard deviation angle in rad
    std_vipz_ = 0.005;


    is_initialized_ = false;

    P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

    x_.fill(0.0);
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);


    R_encoder_ = MatrixXd(2, 2);
    R_encoder_ << std_ent1_*std_ent1_, 0,
            0, std_ent2_*std_ent2_;

    R_vision_ = MatrixXd(3, 3);
    R_vision_ << std_vipx_*std_vipx_, 0, 0,
            0, std_vipx_*std_vipx_,0,
            0 ,0 , std_vipz_ *std_vipz_;
}

EKF::~EKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void EKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    if(!use_vision_ && meas_package.sensor_type_ == MeasurementPackage::VISION){
        return;
    }
    if(!use_encoder_ && meas_package.sensor_type_ == MeasurementPackage::ENCODER){
        return;
    }
    if (!is_initialized_) {
        // first measurement
        x_.fill(0.0);

        if (meas_package.sensor_type_ == MeasurementPackage::ENCODER) {
            x_[0] = meas_package.raw_measurements_[0];
            x_[1] = meas_package.raw_measurements_[1];
            x_[2] = 0.84;
        } else { // Vision Init
            float x = meas_package.raw_measurements_[0];
            float y = meas_package.raw_measurements_[1];
            float z = meas_package.raw_measurements_[2];

            float theta1 = atan2(x,z);
            float theta2 = atan2(y,z);

            x_[0] = theta1;
            x_[1] = theta2;
            x_[2] = 0.84;//sqrt(0.859250359444*0.859250359444+0.0274572322272*0.0274572322272+0.105366327853*0.105366327853);
        }
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }
    double delta_t =(meas_package.timestamp_ - time_us_) /  1000000.0;
    time_us_ = meas_package.timestamp_;
    transition_function(delta_t, meas_package);

}

void EKF::transition_function(double delta_t, MeasurementPackage mearPack){

    VectorXd input_x(5);
    input_x = x_;
    VectorXd nextState(5);
    float theta1 = input_x[0];
    float theta2 = input_x[1];
    float l = input_x[2];
    float w1 = input_x[3];
    float w2 = input_x[4];


    VectorXd term2 = VectorXd(5);
    VectorXd term3 = VectorXd(5);
    VectorXd result = VectorXd(5);

    term2 << w1 * delta_t, w2 * delta_t, 0, 0, 0;

    x_ = input_x.head(5) + term2;
    Eigen::MatrixXd JA(5,5);
    JA<< 1, 0,0,delta_t,0,
            0, 1,0,0,delta_t,
            0, 0,1,0 ,0,
            0, 0,0,1,0,
            0, 0,0,0,1;

    MatrixXd G(5,2);
    G<< 0.5*delta_t*delta_t, 0,
            0 , 0.5 * delta_t*delta_t,
            0,0,
            delta_t,0,
            0,delta_t;

    MatrixXd Q_v(2,2);
    Q_v<< std_a1_*std_a1_, 0,
            0, std_a2_*std_a2_;
    MatrixXd Q(5,5);
    Q= G*Q_v*G.transpose();
    P_ = JA*P_*JA.transpose()+Q;
    MatrixXd I(5,5);
    I<< 1,0,0,0,0,
            0,1,0,0,0,
            0,0,1,0,0,
            0,0,0,1,0,
            0,0,0,0,1;
    ///* Measurement Update (Correction)
    if(mearPack.sensor_type_ == MeasurementPackage::ENCODER){
        MatrixXd S;
        MatrixXd H_encoder_(2,5) ;
        H_encoder_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0;
        S = H_encoder_ * P_ * H_encoder_.transpose() + R_encoder_;
        MatrixXd K(5,3);
        K = P_*H_encoder_.transpose()*S.inverse();
        VectorXd z(2);
        z << mearPack.raw_measurements_[0], mearPack.raw_measurements_[1];
        VectorXd y = z - H_encoder_ * x_;
        //angle normalization
        x_ = x_ + K * y;

        P_ = (I-K*H_encoder_)*P_;

        P_ = P_ - K * S * K.transpose();

    }else
    {
        MatrixXd K(5,3);
        Eigen::MatrixXd JH =  jacob(x_[0],x_[1],x_[2]);

        MatrixXd S = JH *P_*JH.transpose() + R_vision_;
        K = P_* JH.transpose() * S.inverse();
        VectorXd mear(3);
        mear = measurement_function(x_[0],x_[1],x_[2]);

        VectorXd z(3);
        z << mearPack.raw_measurements_[0], mearPack.raw_measurements_[1], mearPack.raw_measurements_[2];
        VectorXd y = z - mear;
        //angle normalization
        x_ = x_ + K * y;
        while(x_[0]>M_PI)x_[0]-=2*M_PI;
        while(x_[0]<-M_PI)x_[0]+=2*M_PI;
        while(x_[1]>M_PI)x_[1]-=2*M_PI;
        while(x_[1]<-M_PI)x_[1]+=2*M_PI;
        P_ = (I - K * JH) * P_;

    }

}
