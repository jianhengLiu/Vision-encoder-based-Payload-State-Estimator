//
// Created by kevin on 2020/9/9.
//
//OpenCV
#include <opencv2/opencv.hpp>
#include <iostream>
// STL
#include <memory>
// Eigen
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
std::string config_file = "/home/kevin/UAV/icra_ws/src/waypoint_generator/config/default.yaml";
int main(){

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    cv::Mat readMat;
    fs["waypointMatrix"] >> readMat;
    Eigen::MatrixXd  readMat_e(9,3);
    cv::cv2eigen(readMat,readMat_e);
    std::cout<<readMat_e<<std::endl;


    return 0;
}