//
// Created by kevin on 2020/8/5.
//

#ifndef TRACKER_MATH_H
#define TRACKER_MATH_H
#include "loadController/common_include.h"
namespace wtr{

    class Math{
    private:
        ros::Time time_now;
    public:
        Math(){
            g = -9.8;
            ezI = Vec3(0,0,1);
        }
        double g;
        Vec3 ezI;
        // 计算上次setTime到现在的时间，单位s
        double getPassTime();
        // 设置更新当前时间
        void setTime(ros::Time now);
        // 计算向量差分
        Vec3 getVectorDiff(Vec3 cVector, Vec3 pVector, double delta_t);
        // 重载二阶差分
        Vec3 getVectorDiff(Vec3 cVector, Vec3 pVector, Vec3 ppVector, double delta_t);

        //反对称矩阵运算
        Mat33 vectorToAntisymmetricMatrix(Vec3 vector);
        Vec3 antisymmetricMatrixToVector(Mat33 antisymmetricMatrix);

        // 旋转矩阵误差计算
        Mat33 rotationMatrixError(Mat33 desieredRotationMatrix, Mat33 currentRotationMatrix);

        // 归一化
        Vec3 normalized(Vec3 inputVec3);
    };




}


#endif //TRACKER_MATH_H
