//
// Created by nrsl on 8/9/20.
//

//
// Created by nrsl on 8/9/20.
//
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "eigen3/Eigen/Eigen"
ros::Publisher pubUAV;
ros::Publisher remapLoad;
geometry_msgs::PoseStamped pubPose;
geometry_msgs::PoseStamped pubLoadPose;


void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){

    pubPose.pose.position.x =  msg->transform.translation.x;
    pubPose.pose.position.y =  msg->transform.translation.y;
    pubPose.pose.position.z =  msg->transform.translation.z;
    pubPose.pose.orientation = msg->transform.rotation;

    pubPose.header.stamp = ros::Time::now();
    pubPose.header.frame_id = "base_link";
    pubUAV.publish(pubPose);
}

void viconLoadCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){

    Eigen::Vector3d loadPose_W;
    loadPose_W.x() =  msg->transform.translation.x - pubPose.pose.position.x;// + 0.061221;
    loadPose_W.y() =  msg->transform.translation.y - pubPose.pose.position.y;//-0.0122312;   //原来是带-的
    loadPose_W.z() =  msg->transform.translation.z - pubPose.pose.position.z;//-0.0520627;

    Eigen::Quaterniond quat;
    quat.x() = pubPose.pose.orientation.x;
    quat.y() = pubPose.pose.orientation.y;
    quat.z() = pubPose.pose.orientation.z;
    quat.w() = pubPose.pose.orientation.w;
    Eigen::Vector3d loadPose_E = quat.toRotationMatrix().inverse()*loadPose_W;

    pubLoadPose.pose.position.x =  loadPose_E.x();// + 0.061221;
    pubLoadPose.pose.position.y =  -loadPose_E.y();//-0.0122312;   //原来是带-的
    pubLoadPose.pose.position.z =  -loadPose_E.z();//-0.0520627;
    pubLoadPose.pose.orientation = msg->transform.rotation;

    pubLoadPose.header.stamp = ros::Time::now();
    pubLoadPose.header.frame_id = "map";
    remapLoad.publish(pubLoadPose);
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"rsRemap");
    ros::NodeHandle nh;
    ros::Subscriber viconSub = nh.subscribe("vicon/uav/uav",1,viconCallback);
    ros::Subscriber viconLoadSub = nh.subscribe("vicon/payload/payload",1,viconLoadCallback);
    pubUAV = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    remapLoad = nh.advertise<geometry_msgs::PoseStamped>("/uav/load/estimation/encoder",1);


    ros::spin();
    return 0;
}
