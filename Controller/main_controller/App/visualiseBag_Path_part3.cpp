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
ros::Publisher pubDesired;

ros::Publisher pubUAV_Path;
ros::Publisher pubGroundTruth_Path;
ros::Publisher pubEncoder_Path;

nav_msgs::Path UAV_Path;
nav_msgs::Path GroundTruth_Path;
nav_msgs::Path Encoder_Path;


geometry_msgs::PoseStamped UAV;
geometry_msgs::PoseStamped GroundTruth;
geometry_msgs::PoseStamped Desired;

bool isDeisred = false;
void mavrosUavCallback(const nav_msgs::OdometryConstPtr &msg){
    UAV.header = msg->header;
    UAV.pose = msg->pose.pose;

    if(msg->header.stamp.toSec())
    pubUAV.publish(UAV);
    UAV_Path.poses.push_back(UAV);
    pubUAV_Path.publish(UAV_Path);
}

void desiredLoadPosCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    isDeisred =true;
    Desired.header = msg->header;
    Desired.pose = msg->pose;

    pubDesired.publish(Desired);
    Encoder_Path.poses.push_back(Desired);
    pubEncoder_Path.publish(Encoder_Path);
}

void viconLoadCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){
    GroundTruth.header = msg->header;

    int delta_x = 2;
    int delta_y = 2;
    int delta_z = 2;

//    if(isDeisred)
//    {
//        GroundTruth.pose.position.x = msg->transform.translation.x + (Desired.pose.position.x-msg->transform.translation.x)/delta_x+0.35;
//        GroundTruth.pose.position.y = msg->transform.translation.y + (Desired.pose.position.y-msg->transform.translation.y)/delta_y-0.1;
//        GroundTruth.pose.position.z = msg->transform.translation.z + (Desired.pose.position.z-msg->transform.translation.z)/delta_z;
//
//
//    }
//    else
    {
        GroundTruth.pose.position.x = msg->transform.translation.x;
        GroundTruth.pose.position.y = msg->transform.translation.y;
        GroundTruth.pose.position.z = msg->transform.translation.z;
    }
    pubGroundTruth.publish(GroundTruth);
    GroundTruth_Path.poses.push_back(GroundTruth);
    pubGroundTruth_Path.publish(GroundTruth_Path);
}



int main(int argc, char ** argv){
    ros::init(argc,argv,"visualiseBag_part2");
    ros::NodeHandle nh;
    ros::Subscriber sub_desired_payload = nh.subscribe("payloadPos",1,desiredLoadPosCallback);
    ros::Subscriber sub_vicon_payload = nh.subscribe("vicon/payload/payload",1,viconLoadCallback);
    ros::Subscriber sub_mavros_uav = nh.subscribe("mavros/local_position/odom",1,mavrosUavCallback);


    pubUAV = nh.advertise<geometry_msgs::PoseStamped>("GroundTruth_uav", 1);
    pubGroundTruth = nh.advertise<geometry_msgs::PoseStamped>("GroundTruth",1);
    pubDesired = nh.advertise<geometry_msgs::PoseStamped>("Desired",1);

    pubUAV_Path = nh.advertise<nav_msgs::Path>("GroundTruth_uav_Path", 1);
    pubGroundTruth_Path = nh.advertise<nav_msgs::Path>("GroundTruth_Path",1);
    pubEncoder_Path = nh.advertise<nav_msgs::Path>("Encoder_Path", 1);

    UAV_Path.header.frame_id = "map";
    UAV_Path.header.stamp = ros::Time::now();

    GroundTruth_Path = UAV_Path;
    Encoder_Path = UAV_Path;


    ros::spin();
    return 0;
}
