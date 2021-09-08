//
// Created by kevin on 2020/8/5.
//

#include <loadController/math.h>

namespace wtr{

        // 计算向量差分
        Vec3 Math::getVectorDiff(Vec3 cVector, Vec3 pVector, double delta_t ){
            return (cVector - pVector)/delta_t;
        }
        // 重载二阶差分
        Vec3 Math::getVectorDiff(Vec3 cVector, Vec3 pVector, Vec3 ppVector, double delta_t){
            return (cVector - 2 * pVector + ppVector)/delta_t/delta_t;
        }

        //反对称矩阵运算
        Mat33 Math::vectorToAntisymmetricMatrix(Vec3 vector){
            Mat33 antisymmetricMatrix;
            antisymmetricMatrix << 0, -vector.z(), vector.y(),
                    vector.z(), 0, -vector.x(),
                    -vector.y(), vector.x(), 0;
            return antisymmetricMatrix;
        };

        Vec3 Math::antisymmetricMatrixToVector(Mat33 antisymmetricMatrix){
            Vec3 vector(antisymmetricMatrix(7), antisymmetricMatrix(2), antisymmetricMatrix(3));
            return vector;
        }

        // 旋转矩阵误差计算
//        Mat33 Math::rotationMatrixError(Mat33 desieredRotationMatrix, Mat33 currentRotationMatrix){
//            Mat33 errorRotationMatrix = 0.5 * vectorToAntisymmetricMatrix(
//                    desieredRotationMatrix.transpose() * currentRotationMatrix
//                    -  currentRotationMatrix.transpose() * desieredRotationMatrix
//            );
//            return errorRotationMatrix;
//        }

        double Math::getPassTime() {
            ros::Time now = ros::Time::now();
            double now_sec = (double)(now.nsec) *  1e-9 + (double)(now.sec);
            double last_sec =  (double)(time_now.nsec) *  1e-9 + (double)(time_now.sec);
            return now_sec - last_sec;
        }

        void Math::setTime(ros::Time now) {
            time_now = now;
        }

        Vec3 Math::normalized(Vec3 inputVec3) {
            return inputVec3 / inputVec3.norm();
        }




}