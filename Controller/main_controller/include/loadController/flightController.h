//
// Created by chrisliu on 2020/2/26.
//

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H
#include "loadController/math.h"
#include "loadController/common_include.h"
#include "loadController/dataStructure.h"

#define PI 3.1415926535
class PayloadController
{
public:
    Vec3 baro;
    Vec3 baroAlter;
    Vec3 cPayloadVectorBaro;

    wtr::Math math;

 	bool isDrop = false;

    nav_msgs::Odometry desieredLoadPos;

    nav_msgs::Odometry desieredPos;

    nav_msgs::Odometry feedbackLoad;

    double past;

    Vec3 accumulateError;

    // 初始化无人机与负载参数
    void initializeParameter(double inputMassQuadcopter, double inputMassPayload,double inputLength, Mat33 inertial);
    // 进行一次飞控计算
//    void runControlOnce(Vec3 desieredPayloadPosition, Vec3 desieredPayloadVelocity, Vec3 desieredPayloadAcceleration);
    void runControlOnce(Vec3 desieredPayloadPos, Vec3 desieredPayloadVel, Vec3 desieredPayloadAcc,Vec3 desieredPayloadJerk,Vec3 desieredPayloadSnap,Vec3 desieredD5);

    // 更新无人机状态
    void updateImu(const sensor_msgs::ImuConstPtr imu_msg);

    void updateOdom(const nav_msgs::OdometryConstPtr odom_msg);

    void updatePayload(const geometry_msgs::PoseStampedConstPtr payload_msg);

    void updatePayload(geometry_msgs::PoseStamped payload_msg);

    Vec4 getRevs();

    Vec4 getRevs(Eigen::Vector3d inputDesiredPos,
                 Eigen::Vector3d inputDesiredVel,
                 Eigen::Vector3d inputDesiredAcc,
                 Eigen::Vector3d inputDesiredJerk);

    double getLiftForce();

    geometry_msgs::Quaternion getQuaternion();

    void LiuController(Vec3 desieredPayloadPos,Vec3 desieredPayloadVel,Vec3 desieredPayloadAcc,Vec3 desieredPayloadJerk,Vec3 desieredPayloadSnap);

private:
    int cnt;
    bool first_time = true;

    void propellerController();

public:
    // 无人机质量
    double massQuadcopter;
    // 无人机转动惯量矩阵
    Mat33 II;
    // 负载质量
    double massPayload;
    // 绳子长度
    double length;
    // 姿态控制输出油门
    double outputThrust;
    double force;
    // 时间
    int time;
    //位置环计数器
    int cntPos;

    // 差分中的时间变量
    double dt;

    // 输出三轴动量
    Vec3 moment;


    /**
    * @brief 反馈参数
    */
    Feedback_t feedback;

    Calculate_t calculate;

    Desiered_t desiered;

    Output_t output;
};


#endif //FLIGHT_CONTROLLER_H
