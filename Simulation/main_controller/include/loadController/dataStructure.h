//
// Created by kevin on 2020/8/5.
//

#ifndef LOADCONTROLLER_DATASTRUCTURE_H
#define LOADCONTROLLER_DATASTRUCTURE_H

#include "loadController/common_include.h"


/**
 * @brief  类型1:
 *         无人机所有反馈数据及其衍生数据，包括
 *          × 无人机位置速度
 *          × 负载位置
 *          × 无人机姿态四元数，角速度，角加速度
 */
typedef struct{
    Vec3 cBodyPosition;
    Vec3 cBodyVelocity;
    Vec3 cBodyAcc;

    // 机体坐标系下的负载位置
    Vec3 cPayloadPos_B;

    Vec3 cPayloadPosition;
    Vec3 pPayloadPosition;
    Vec3 cPayloadVelocity;
    Vec3 cPayloadAcc;

    // 反馈四元数
    Eigen::Quaterniond cBodyQuaternion;
    Eigen::Matrix3d cBodyRotationMatrix;
    // 机身角速度
    Vec3 cBodyAngularVelocity;
    Vec3 pBodyAngularVelocity;
    Vec3 ppBodyAngularVelocity;

}Feedback_t;


/**
 * @brief  类型2:
 *         无人机所有计算状态
 *          × 无人机加速度
 *          × 负载速度，负载加速度
 *          × 无人机姿态欧拉角，旋转矩阵
 *          * 负载的拉力方向向量
 */

typedef struct{
    // 机身位置速度加速度
    Vec3 pBodyPosition;

    Vec3 pBodyVelocity;

    Vec3 cBodyAcceleration;


    // 历负载位置速度加速度
    Vec3 cPayloadPosition;
    Vec3 pPayloadPosition;

    Vec3 cPayloadVelocity;
    Vec3 pPayloadVelocity;

    Vec3 cPayloadAcceleration;

    // 机身角速度
    Vec3 cBodyAngularVelocity;
    Vec3 pBodyAngularVelocity;
    Vec3 ppBodyAngularVelocity;

    Mat33 cRotationMatrix;
    Mat33 pRotationMatrix;
    Mat33 ppRotationMatrix;


    // 负载拉力方向向量
    Vec3 cPayloadVector;
    Vec3 pPayloadVector;
    Vec3 ppPayloadVector;
    // 负载拉力方向向量的变化率
    Vec3 cPayloadVectorD;
    Vec3 pPayloadVectorD;

    Vec3 cPayloadVectorDD;

    // 负载拉力方向向量
    Vec3 cPayloadOrientation;
    Vec3 pPayloadOrientation;
    Vec3 ppPayloadOrientation;
    // 负载拉力方向向量的变化率
    Vec3 cPayloadOrientationD;
    Vec3 cPayloadOrientationDD;

    Vec3 cBodyAngularAcceleration;
    Vec3 pBodyAngularAcceleration;

    Vec3 accumulatePayloadPositionError;
    Vec3 accumulateBodyPositionError;

    Vec3 A;


}Calculate_t;

/**
 * @brief  类型3:
 *         无人机所有期望状态：
 *
 */

typedef struct{
    /**
     * @brief  负载
     */
    Vec3 cPayloadPosition;
    Vec3 pPayloadPosition;
    Vec3 ppPayloadPosition;

    Vec3 cPayloadVelocity;
    Vec3 cPayloadAcceleration;
    Vec3 cPayloadJerk;
    Vec3 cPayloadSnap;


    Vec3 cPayloadVector;
    Vec3 pPayloadVector;
    Vec3 ppPayloadVector;
    Vec3 cPayloadVectorD;
    Vec3 cPayloadVectorDD;

    Vec3 cPayloadOrientation;
    Vec3 pPayloadOrientation;
    Vec3 ppPayloadOrientation;
    Vec3 cPayloadOrientationD;
    Vec3 pPayloadOrientationD;

    Vec3 cPayloadOrientationDD;
    /**
     * @brief  机身
     */
    Mat33 cRotationMatrix;
    Vec3 cBodyPosition;
    Vec3 pBodyPosition;
    Vec3 cBodyVelocity;
    Eigen::Quaterniond cBodyQuat;
    double yaw;

    Vec3 cBodyEulerAngle;
    Vec3 pBodyEulerAngle;
    Vec3 ppBodyEulerAngle;
    Vec3 cBodyAngularVelocity;
    Vec3 cBodyAngularAcceleration;

    Vec3 forceVector;

    /**
     * 期望状态bianliang
     *
     * d(n)Tp:(n)次导_拉力_payload
     * q:负载期望单位向量
     *
     */
    Vec3 Tp;
    double norm_Tp;
    Vec3 q;

    Vec3 dTp;
    double dnorm_Tp;
    Vec3 dq;

    Vec3 d2Tp;
    double d2norm_Tp;
    Vec3 d2q;

    Vec3 d3Tp;
    double d3norm_Tp;
    Vec3 d3q;

    Vec3 omega;
    Vec3 domega;
    /**
     *
     */
     Vec3 cWd;
     Vec3 pWd;
     Vec3 dWd;
}Desiered_t;

typedef struct {
    Vec3 M;
    Vec3 eulerAngular;
    double thrust;
    geometry_msgs::Quaternion quaternion;
    Vec4 revs;

}Output_t;

#endif //LOADCONTROLLER_DATASTRUCTURE_H
