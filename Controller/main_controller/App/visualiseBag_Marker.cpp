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
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include "eigen3/Eigen/Eigen"
ros::Publisher pubUAV;
ros::Publisher pubGroundTruth;
ros::Publisher pubEncoder;
ros::Publisher pubVision;

visualization_msgs::Marker UAV_Marker;
visualization_msgs::Marker GroundTruth_Marker;
visualization_msgs::Marker Encoder_Marker;
visualization_msgs::Marker Vision_Marker;

geometry_msgs::Point UAV;
geometry_msgs::Point GroundTruth;
geometry_msgs::Point Encoder;
geometry_msgs::Point Vision;

Eigen::Quaterniond quat;
void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){
    UAV.x = msg->transform.translation.x;
    UAV.y = msg->transform.translation.y;
    UAV.z = msg->transform.translation.z;

    quat.x() =  msg->transform.rotation.x;
    quat.y() = msg->transform.rotation.y;
    quat.z() = msg->transform.rotation.z;
    quat.w() = msg->transform.rotation.w;

    UAV_Marker.points.push_back(UAV);
    pubUAV.publish(UAV_Marker);
}

void viconLoadCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){
    GroundTruth.x = msg->transform.translation.x;
    GroundTruth.y = msg->transform.translation.y;
    GroundTruth.z = msg->transform.translation.z;

    GroundTruth_Marker.points.push_back(GroundTruth);
    pubGroundTruth.publish(GroundTruth_Marker);
}

void encoderCallback(const geometry_msgs::PoseStampedConstPtr &msg){

    Eigen::Vector3d cPayloadPos_B(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    Eigen::Matrix3d rotationMatrix_C2B;
    //camera to body:alpha=-180,beta = zeta = 0
    //崔健P25
    rotationMatrix_C2B << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    Eigen::Vector3d cPayloadVector = quat.toRotationMatrix() * rotationMatrix_C2B * cPayloadPos_B;



    Encoder.x = UAV.x + cPayloadVector.x();
    Encoder.y = UAV.y + cPayloadVector.y();
    Encoder.z = UAV.z + cPayloadVector.z();

    Encoder_Marker.points.push_back(Encoder);
    pubEncoder.publish(Encoder_Marker);
}

void cameraCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    Eigen::Vector3d cPayloadPos_B(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    Eigen::Matrix3d rotationMatrix_C2B;
    //camera to body:alpha=-180,beta = zeta = 0
    //崔健P25
    rotationMatrix_C2B << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    Eigen::Vector3d cPayloadVector = quat.toRotationMatrix() * rotationMatrix_C2B * cPayloadPos_B;



    Vision.x = UAV.x + cPayloadVector.x();
    Vision.y = UAV.y + cPayloadVector.y();
    Vision.z = UAV.z + cPayloadVector.z();

    Vision_Marker.points.push_back(Vision);
    pubVision.publish(Vision_Marker);
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"visualiseBag");
    ros::NodeHandle nh;
    ros::Subscriber viconSub = nh.subscribe("vicon/encoder/encoder",1,viconCallback);
    ros::Subscriber viconLoadSub = nh.subscribe("vicon/payload/payload",1,viconLoadCallback);

    ros::Subscriber subEncoder = nh.subscribe("/uav/load/estimation/encoder_real",1,encoderCallback);
    ros::Subscriber subVision = nh.subscribe("/payload_camera",1,cameraCallback);

    pubUAV = nh.advertise<visualization_msgs::Marker>("GroundTruth_uav", 1);
    pubGroundTruth = nh.advertise<visualization_msgs::Marker>("GroundTruth",1);
    pubEncoder = nh.advertise<visualization_msgs::Marker>("Encoder",1);
    pubVision = nh.advertise<visualization_msgs::Marker>("Vision",1);

    UAV_Marker.header.stamp = ros::Time::now();
    UAV_Marker.header.frame_id = "map";

    UAV_Marker.ns = "traj_node/trajectory_waypoints";
    UAV_Marker.id = 0;
    UAV_Marker.type = visualization_msgs::Marker::SPHERE_LIST;
    UAV_Marker.action = visualization_msgs::Marker::ADD;
    UAV_Marker.scale.x = 0.05;
    UAV_Marker.scale.y = 0.05;
    UAV_Marker.scale.z = 0.05;
    UAV_Marker.pose.orientation.x = 0.0;
    UAV_Marker.pose.orientation.y = 0.0;
    UAV_Marker.pose.orientation.z = 0.0;
    UAV_Marker.pose.orientation.w = 1.0;

    UAV_Marker.color.a = 1.0;
    UAV_Marker.points.clear();

    GroundTruth_Marker = UAV_Marker;
    Encoder_Marker = UAV_Marker;
    Vision_Marker = UAV_Marker;

    UAV_Marker.color.r = 0.0;
    UAV_Marker.color.g = 1.0;
    UAV_Marker.color.b = 1.0;

    GroundTruth_Marker.color.r = 0.0;
    GroundTruth_Marker.color.g = 1.0;
    GroundTruth_Marker.color.b = 0.0;

    Encoder_Marker.color.r = 1.0;
    Encoder_Marker.color.g = 0.0;
    Encoder_Marker.color.b = 0.0;

    Vision_Marker.color.r = 0.0;
    Vision_Marker.color.g = 0.0;
    Vision_Marker.color.b = 1.0;



    ros::spin();
    return 0;
}
