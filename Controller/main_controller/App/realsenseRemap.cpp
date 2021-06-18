//
// Created by nrsl on 8/9/20.
//

//
// Created by nrsl on 8/9/20.
//
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
ros::Publisher pubUAV;
double vx,vy;
geometry_msgs::PoseStamped pubPose;
void rsCallback(const nav_msgs::Odometry &msg){

    pubPose.pose = msg.pose.pose;
    pubPose.header.stamp = ros::Time::now();
    pubPose.header.frame_id = "base_link";
    pubUAV.publish(pubPose);
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"rsRemap");
    ros::NodeHandle nh;
    ros::Subscriber joySub;
    joySub = nh.subscribe("/camera/odom/sample", 1,rsCallback);
    pubUAV = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    ros::spin();
    return 0;
}
