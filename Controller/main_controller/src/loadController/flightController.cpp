//
// Created by Kevin on 2020/8/05.
//

#include "loadController/flightController.h"

using namespace std;

void PayloadController::updateImu(const sensor_msgs::ImuConstPtr imu_msg) {
    feedback.cBodyQuaternion = Eigen::Quaterniond(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    feedback.cBodyAcc = Vec3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
    feedback.cBodyAngularVelocity = Vec3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

    feedback.cBodyRotationMatrix = feedback.cBodyQuaternion.toRotationMatrix();
}

void PayloadController::updateOdom(const nav_msgs::OdometryConstPtr odom_msg){
    feedback.cBodyPosition = Vec3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    feedback.cBodyVelocity = Vec3(odom_msg->twist.twist.linear.x,odom_msg->twist.twist.linear.y,odom_msg->twist.twist.linear.z);
}

void PayloadController::updatePayload(const geometry_msgs::PoseStampedConstPtr payload_msg){
    dt = payload_msg->header.stamp.toSec() - past;
    past = payload_msg->header.stamp.toSec();

    /**
     * 反馈的是机体坐标系下的负载位置
     */
    feedback.cPayloadPos_B = Vec3(payload_msg->pose.position.x, payload_msg->pose.position.y, payload_msg->pose.position.z);



    Eigen::Matrix3d rotationMatrix_C2B;
    //camera to body:alpha=-180,beta = zeta = 0
    //崔健P25
    rotationMatrix_C2B << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    Vec3 cPayloadVector = feedback.cBodyRotationMatrix * rotationMatrix_C2B * feedback.cPayloadPos_B;
    feedback.pPayloadPosition = feedback.cPayloadPosition;
    feedback.cPayloadPosition = feedback.cBodyPosition+cPayloadVector;

    calculate.pPayloadVector = calculate.cPayloadVector;
    calculate.cPayloadVector = cPayloadVector;
    calculate.cPayloadVectorD = math.getVectorDiff(calculate.cPayloadVector,calculate.pPayloadVector,dt);

    double alpha = 0.5;
    Vec3 cPayloadVelocity = feedback.cBodyVelocity + calculate.cPayloadVectorD;
    feedback.cPayloadVelocity = alpha*cPayloadVelocity +(1-alpha)* feedback.cPayloadVelocity;

//    feedback.cPayloadVelocity = feedback.cBodyVelocity + calculate.cPayloadVectorD;//math.getVectorDiff(feedback.cPayloadPosition,feedback.pPayloadPosition,dt);

    feedbackLoad.header.frame_id = "map";
    feedbackLoad.header.stamp = ros::Time::now();

    feedbackLoad.pose.pose.position.x = feedback.cPayloadPosition.x();
    feedbackLoad.pose.pose.position.y = feedback.cPayloadPosition.y();
    feedbackLoad.pose.pose.position.z = feedback.cPayloadPosition.z();

    feedbackLoad.twist.twist.linear.x = feedback.cPayloadVelocity.x();
    feedbackLoad.twist.twist.linear.y = feedback.cPayloadVelocity.y();
    feedbackLoad.twist.twist.linear.z = feedback.cPayloadVelocity.z();

    calculate.cPayloadOrientation = calculate.cPayloadVector.normalized();
    calculate.cPayloadOrientationD = calculate.cPayloadVectorD.normalized();// feedback.cBodyAngularVelocity.cross(calculate.cPayloadOrientation);

}

void PayloadController::updatePayload(geometry_msgs::PoseStamped payload_msg){
    dt = payload_msg.header.stamp.toSec() - past;
    past = payload_msg.header.stamp.toSec();

    /**
     * 反馈的是机体坐标系下的负载位置
     */
    feedback.cPayloadPos_B = Vec3(payload_msg.pose.position.x, payload_msg.pose.position.y, payload_msg.pose.position.z);



    Eigen::Matrix3d rotationMatrix_C2B;
    //camera to body:alpha=-180,beta = zeta = 0
    //崔健P25
    rotationMatrix_C2B << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    Vec3 cPayloadVector = feedback.cBodyRotationMatrix * rotationMatrix_C2B * feedback.cPayloadPos_B;

    calculate.pPayloadVector = calculate.cPayloadVector;

    double a = 0.5;
    calculate.cPayloadVector = a * cPayloadVector  + (1.0f - a) * calculate.cPayloadVector;

//    calculate.cPayloadVector = cPayloadVector;

    calculate.cPayloadVectorD = math.getVectorDiff(calculate.cPayloadVector,calculate.pPayloadVector,dt);


    feedback.pPayloadPosition = feedback.cPayloadPosition;
    feedback.cPayloadPosition = feedback.cBodyPosition+cPayloadVector;




    feedback.cPayloadVelocity = feedback.cBodyVelocity + calculate.cPayloadVectorD;//math.getVectorDiff(feedback.cPayloadPosition,feedback.pPayloadPosition,dt);

    feedbackLoad.header.frame_id = "map";
    feedbackLoad.header.stamp = ros::Time::now();

    feedbackLoad.pose.pose.position.x = feedback.cPayloadPosition.x();
    feedbackLoad.pose.pose.position.y = feedback.cPayloadPosition.y();
    feedbackLoad.pose.pose.position.z = feedback.cPayloadPosition.z();

    feedbackLoad.twist.twist.linear.x = feedback.cPayloadVelocity.x();
    feedbackLoad.twist.twist.linear.y = feedback.cPayloadVelocity.y();
    feedbackLoad.twist.twist.linear.z = feedback.cPayloadVelocity.z();

    calculate.cPayloadOrientation = calculate.cPayloadVector.normalized();
    calculate.cPayloadOrientationD = calculate.cPayloadVectorD.normalized();// feedback.cBodyAngularVelocity.cross(calculate.cPayloadOrientation);

}

void PayloadController::propellerController(){
    Eigen::Matrix4d Minvese;
    double sqrt2 = sqrt(2);
    Minvese << 1, -sqrt2, sqrt2, 1,
            1, -sqrt2, -sqrt2, -1,
            1, sqrt2, -sqrt2, 1,
            1, sqrt2, sqrt2, -1;
    Minvese = 0.25 * Minvese;

    Eigen::Vector4d input(output.thrust, output.M.x(), output.M.y(), output.M.z());
    output.revs = Minvese * input;
    if (output.revs.x() < 0) {
        output.revs.x() = 0;
    }
    if (output.revs.y() < 0) {
        output.revs.y() = 0;
    }
    if (output.revs.z() < 0) {
        output.revs.z() = 0;
    }
    if (output.revs.w() < 0) {
        output.revs.w() = 0;
    }
    output.revs.x() = sqrt(output.revs.x());
    output.revs.y() = sqrt(output.revs.y());
    output.revs.z() = sqrt(output.revs.z());
    output.revs.w() = sqrt(output.revs.w());
}

Vec4 PayloadController::getRevs() {
    return output.revs;
}

double PayloadController::getLiftForce()
{
    return output.thrust;
}

geometry_msgs::Quaternion PayloadController::getQuaternion()
{
    return output.quaternion;
}

void PayloadController::initializeParameter(double inputMassQuadcopter, double inputMassPayload, double inputLength, Mat33 inertial) {
    massQuadcopter = inputMassQuadcopter;
    massPayload = inputMassPayload;
    length = inputLength;
    II = inertial;
    desiered.yaw = 0;
}

void PayloadController::LiuController(Vec3 desieredPayloadPos,Vec3 desieredPayloadVel,Vec3 desieredPayloadAcc,Vec3 desieredPayloadJerk,Vec3 desieredPayloadSnap) {
/**
     * 更新期望状态
     */

    desiered.cPayloadPosition = desieredPayloadPos;
    desiered.cPayloadVelocity = desieredPayloadVel;
    desiered.cPayloadAcceleration = desieredPayloadAcc;

    desieredLoadPos.header.frame_id = "map";
    desieredLoadPos.header.stamp = ros::Time::now();

    desieredLoadPos.pose.pose.position.x = desiered.cPayloadPosition.x();
    desieredLoadPos.pose.pose.position.y = desiered.cPayloadPosition.y();
    desieredLoadPos.pose.pose.position.z = desiered.cPayloadPosition.z();

    desieredLoadPos.twist.twist.linear.x = desiered.cPayloadVelocity.x();
    desieredLoadPos.twist.twist.linear.y = desiered.cPayloadVelocity.y();
    desieredLoadPos.twist.twist.linear.z = desiered.cPayloadVelocity.z();


    Eigen::Vector3d e3(Eigen::Vector3d::UnitZ());

    // 负载期望张力
    Vec3 Tp = -massPayload * (desieredPayloadAcc + 9.8 * e3);
    double norm_Tp = Tp.norm();
    desiered.cPayloadOrientation = Tp.normalized();
    // 负载张力方向
    desiered.cBodyPosition = desiered.cPayloadPosition - length*desiered.cPayloadOrientation;


    Vec3 dTp = -massPayload*desieredPayloadJerk;
    double dnorm_Tp = 1/norm_Tp*Tp.dot(dTp);
    desiered.cPayloadOrientationD = (dTp-desiered.cPayloadOrientation*dnorm_Tp)/norm_Tp;
    desiered.cBodyVelocity = desiered.cPayloadVelocity -length*desiered.cPayloadOrientationD;

    Vec3 d2Tp = -massPayload*desieredPayloadSnap;
    double d2norm_Tp = (dTp.dot(dTp) + Tp.dot(d2Tp) - dnorm_Tp*dnorm_Tp)/norm_Tp;
    desiered.cPayloadOrientationDD = (d2Tp-desiered.cPayloadOrientationD*dnorm_Tp-desiered.cPayloadOrientation*d2norm_Tp-desiered.cPayloadOrientationD*dnorm_Tp)/norm_Tp;

    /**
     * 控制部分
     */
    Eigen::Vector3d F;
    if(!isDrop)
    {
        Vec3 errorBodyPos = feedback.cBodyPosition - desiered.cBodyPosition;
        Vec3 errorBodyVel = feedback.cBodyVelocity - desiered.cBodyVelocity;

        Vec3 errorPayloadPosition = feedback.cPayloadPosition - desiered.cPayloadPosition;
        Vec3 errorPayloadVelocity = feedback.cPayloadVelocity - desiered.cPayloadVelocity;

        accumulateError += errorBodyPos;

        float delta = 1;
        const Eigen::Vector3d k_bx = Eigen::Vector3d(1.8, 1.8, 3)/delta;//(2.0, 2.0, 1.5);//(2.0, 2.0, 1.5);
        const Eigen::Vector3d k_bv = Eigen::Vector3d(2.2, 2.2, 1.5)/delta;//(1.8, 1.8, 1);
        const Eigen::Vector3d k_px = Eigen::Vector3d(-0.5,-0.5,1)/delta;//(-2,-2,-1);
        const Eigen::Vector3d k_pv = Eigen::Vector3d(-0.25,-0.25,1)/delta;//(-2.5,-2.5,-1);
        const Eigen::Vector3d k_i = Eigen::Vector3d(0.0001,0.0001,0.0002);

//        const Eigen::Vector3d k_bx = Eigen::Vector3d(1, 1, 2);//(2.0, 2.0, 1.5);
//        const Eigen::Vector3d k_bv = Eigen::Vector3d(1.5, 1.5, 1.5);
//        const Eigen::Vector3d k_px = Eigen::Vector3d(2,2,1);//(-1.2,-1.2,-0.8);//(-0.8, -0.8, -12);//(0,0,0);//
//        const Eigen::Vector3d k_pv = Eigen::Vector3d(2,2,1);//(-2.5,-2.5,-1);//(-2.5,-2.5,-0.5);//(-1.5, -1.5, -2.5);//(1.5, 1.5, 2.5);
//        const Eigen::Vector3d k_i = Eigen::Vector3d(0.0000,0.0000,0.0);//(-4,-4,-1);//(-2.5,-2.5,-1);//(-2.5,-2.5,-0.5);//(-1.5, -1.5, -2.5);//(1.5, 1.5, 2.5);

        Eigen::Vector3d F_n =
                - errorBodyPos.cwiseProduct(k_bx)
                - errorBodyVel.cwiseProduct(k_bv)
                - errorPayloadPosition.cwiseProduct(k_px)
                - errorPayloadVelocity.cwiseProduct(k_pv)
                -  accumulateError.cwiseProduct(k_i)
                +  (massQuadcopter + massPayload) * desiered.cPayloadAcceleration
                +  massQuadcopter*length*(calculate.cPayloadOrientationD.dot(calculate.cPayloadOrientationD)*calculate.cPayloadOrientation);

        F_n += (massQuadcopter + massPayload) * (e3 * 9.8);

        Eigen::Matrix3d antisymmetricMatrix = math.vectorToAntisymmetricMatrix(calculate.cPayloadOrientation);
        Eigen::Vector3d orientationError = antisymmetricMatrix * antisymmetricMatrix * desiered.cPayloadOrientation;

        Eigen::Vector3d orientationErrorD =
                calculate.cPayloadOrientationD - (desiered.cPayloadOrientation.cross(desiered.cPayloadOrientationD)).cross(calculate.cPayloadOrientation);

        const Eigen::Vector3d k_q = Eigen::Vector3d(-1.5, -1.5, -1.5)/delta;//(-2.5,-2.5,0);//(-2.5,-2.5,0);//
        const Eigen::Vector3d k_w = Eigen::Vector3d(0.07,0.07,0.07)/delta;// //p d controller parameters

        Eigen::Vector3d F_pd = -orientationError.cwiseProduct(k_q) - orientationErrorD.cwiseProduct(k_w);

        Eigen::Vector3d F_ff =
                massQuadcopter * length * calculate.cPayloadOrientation.dot((desiered.cPayloadOrientation.cross(desiered.cPayloadOrientationD))) *
                calculate.cPayloadOrientation.cross(calculate.cPayloadOrientationD)
                + massQuadcopter * length *
                  ((desiered.cPayloadOrientation.cross(desiered.cPayloadOrientationDD)).cross(calculate.cPayloadOrientation));

        /**
         * F_pd 影响了稳定负载的能力，但会
         */
        F = F_n - F_pd -F_ff;
    }
    else
    {
        desiered.cPayloadPosition.z() += length;

        Vec3 errorBodyPos = feedback.cBodyPosition - desiered.cPayloadPosition;
        Vec3 errorBodyVel = feedback.cBodyVelocity - desiered.cPayloadVelocity;

        accumulateError += errorBodyPos;

        const Eigen::Vector3d k_bx = Eigen::Vector3d(1, 1, 2);//(2.0, 2.0, 1.5);//(2.0, 2.0, 1.5);
        const Eigen::Vector3d k_bv = Eigen::Vector3d(0.75, 0.75, 1.5);//(1.8, 1.8, 1);
        const Eigen::Vector3d k_i = Eigen::Vector3d(0.0001,0.0001,0.0001);


        Eigen::Vector3d F_n = -errorBodyPos.cwiseProduct(k_bx) - errorBodyVel.cwiseProduct(k_bv)
                              //                              -  accumulateError.cwiseProduct(k_i)
                              +  (massQuadcopter) * desiered.cPayloadAcceleration;

        F_n += (massQuadcopter+0.2) * (e3 * 9.8);

        F = F_n;
    }

    //2,1,0->ZYX
    Eigen::Vector3d b1_des(cos(0), sin(0), 0);

    Eigen::Vector3d b3_des = F / F.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des /= b2_des.norm();

    Eigen::Matrix3d desiredRotationMatrix;
    desiredRotationMatrix.col(0) = b2_des.cross(b3_des);
    desiredRotationMatrix.col(1) = b2_des;
    desiredRotationMatrix.col(2) = b3_des;

    Eigen::Vector3d eulerAngle = desiredRotationMatrix.eulerAngles(0,1,2);

//    double limit_angle = 30*PI/180;
//    if(eulerAngle[0]>limit_angle)
//        eulerAngle[0] = limit_angle;
//    if(eulerAngle[1]>limit_angle)
//        eulerAngle[1] = limit_angle;
    output.quaternion = tf::createQuaternionMsgFromRollPitchYaw(eulerAngle[0],eulerAngle[1],eulerAngle[2]);

    desieredPos.header.frame_id = "map";
    desieredPos.header.stamp = ros::Time::now();

    desieredPos.pose.pose.position.x = desiered.cBodyPosition.x();
    desieredPos.pose.pose.position.y = desiered.cBodyPosition.y();
    desieredPos.pose.pose.position.z = desiered.cBodyPosition.z();
    desieredPos.pose.pose.orientation = output.quaternion;

    desieredPos.twist.twist.linear.x = desiered.cBodyVelocity.x();
    desieredPos.twist.twist.linear.y = desiered.cBodyVelocity.y();
    desieredPos.twist.twist.linear.z = desiered.cBodyVelocity.z();

    Eigen::Vector3d errorRotation =
            0.5 * math.antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * feedback.cBodyRotationMatrix -
                                                    feedback.cBodyRotationMatrix.transpose() * desiredRotationMatrix));

    Eigen::Vector3d errorAngular = feedback.cBodyAngularVelocity;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;

    const Eigen::Vector3d k_p = Eigen::Vector3d(3, 3, 1);
    const Eigen::Vector3d k_d = Eigen::Vector3d(0.8, 0.8, 1);
    output.M = -errorRotation.cwiseProduct(k_p) + errorAngular.cwiseProduct(k_d);// + cAngularVelocity_body.cross(J_q * cAngularVelocity_body);

    output.thrust = b3_des.transpose() * F;
#ifdef VREP
    propellerController();
#endif
}

