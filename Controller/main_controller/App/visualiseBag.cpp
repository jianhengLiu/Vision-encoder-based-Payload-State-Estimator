//
// Created by nrsl on 8/9/20.
//

//
// Created by nrsl on 8/9/20.
//
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"

#include "eigen3/Eigen/Eigen"
ros::Publisher pubUAV;
ros::Publisher pubGroundTruth;
ros::Publisher pubEncoder;
ros::Publisher pubVision;


nav_msgs::Odometry UAV;
nav_msgs::Odometry GroundTruth;
nav_msgs::Odometry Encoder;
nav_msgs::Odometry Vision;
void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){
    UAV.header = msg->header;

    UAV.pose.pose.position.x = msg->transform.translation.x;
    UAV.pose.pose.position.y = msg->transform.translation.y;
    UAV.pose.pose.position.z = msg->transform.translation.z;

    UAV.pose.pose.orientation = msg->transform.rotation;

    pubUAV.publish(UAV);
}

void viconLoadCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){

    GroundTruth.header= msg->header;

    GroundTruth.pose.pose.position.x = msg->transform.translation.x;
    GroundTruth.pose.pose.position.y = msg->transform.translation.y;
    GroundTruth.pose.pose.position.z = msg->transform.translation.z;

    GroundTruth.pose.pose.orientation = msg->transform.rotation;

    pubGroundTruth.publish(GroundTruth);
}

void encoderCallback(const geometry_msgs::PoseStampedConstPtr &msg){

    Eigen::Vector3d cPayloadPos_B(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    Eigen::Quaterniond quat;
    quat.x() = UAV.pose.pose.orientation.x;
    quat.y() = UAV.pose.pose.orientation.y;
    quat.z() = UAV.pose.pose.orientation.z;
    quat.w() = UAV.pose.pose.orientation.w;

    Eigen::Matrix3d rotationMatrix_C2B;
    //camera to body:alpha=-180,beta = zeta = 0
    //崔健P25
    rotationMatrix_C2B << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    Eigen::Vector3d cPayloadVector = quat.toRotationMatrix() * rotationMatrix_C2B * cPayloadPos_B;


    Encoder.header = msg->header;

    Encoder.pose.pose.position.x = UAV.pose.pose.position.x + cPayloadVector.x();
    Encoder.pose.pose.position.y = UAV.pose.pose.position.y + cPayloadVector.y();
    Encoder.pose.pose.position.z = UAV.pose.pose.position.z + cPayloadVector.z();

    pubEncoder.publish(Encoder);
}

void cameraCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    Eigen::Vector3d cPayloadPos_B(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    Eigen::Quaterniond quat;
    quat.x() = UAV.pose.pose.orientation.x;
    quat.y() = UAV.pose.pose.orientation.y;
    quat.z() = UAV.pose.pose.orientation.z;
    quat.w() = UAV.pose.pose.orientation.w;

    Eigen::Matrix3d rotationMatrix_C2B;
    //camera to body:alpha=-180,beta = zeta = 0
    //崔健P25
    rotationMatrix_C2B << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    Eigen::Vector3d cPayloadVector = quat.toRotationMatrix() * rotationMatrix_C2B * cPayloadPos_B;


    Vision.header = msg->header;

    Vision.pose.pose.position.x = UAV.pose.pose.position.x + cPayloadVector.x();
    Vision.pose.pose.position.y = UAV.pose.pose.position.y + cPayloadVector.y();
    Vision.pose.pose.position.z = UAV.pose.pose.position.z + cPayloadVector.z();

    pubVision.publish(Vision);
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"visualiseBag");
    ros::NodeHandle nh;
    ros::Subscriber viconSub = nh.subscribe("vicon/encoder/encoder",1,viconCallback);
    ros::Subscriber viconLoadSub = nh.subscribe("vicon/payload/payload",1,viconLoadCallback);

    ros::Subscriber subEncoder = nh.subscribe("/uav/load/estimation/encoder_real",1,encoderCallback);
    ros::Subscriber subVision = nh.subscribe("/payload_camera",1,cameraCallback);

    pubUAV = nh.advertise<nav_msgs::Odometry>("GroundTruth_uav", 1);
    pubGroundTruth = nh.advertise<nav_msgs::Odometry>("GroundTruth",1);
    pubEncoder = nh.advertise<nav_msgs::Odometry>("Encoder",1);
    pubVision = nh.advertise<nav_msgs::Odometry>("Vision",1);




    ros::spin();
    return 0;
}
