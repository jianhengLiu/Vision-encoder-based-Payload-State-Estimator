//
// Created by nrsl on 8/9/20.
//

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include "eigen3/Eigen/Eigen"

#include "fstream"

using namespace std;
ros::Publisher pubUAV;
ros::Publisher pubGroundTruth;
ros::Publisher pubDesired;

ros::Publisher pubUAV_Path;
ros::Publisher pubGroundTruth_Path;
ros::Publisher pubEncoder_Path;

ros::Publisher pubFeedback;
ros::Publisher pubPayload;

nav_msgs::Path UAV_Path;
nav_msgs::Path GroundTruth_Path;
nav_msgs::Path Encoder_Path;

ros::Publisher pubObstacleArray;

geometry_msgs::PoseStamped GroundTruth_UAV;
geometry_msgs::PoseStamped Desired_UAV;
geometry_msgs::PoseStamped GroundTruth;
geometry_msgs::PoseStamped Desired;

nav_msgs::Odometry Feedback;

visualization_msgs::Marker Payload;
visualization_msgs::MarkerArray ObstacleArray;

double error = 0;
long cnt = 0;
ofstream fout("/home/chrisliu/NewDisk/ROSws/ICRA2021Test_ws/src/Controller/main_controller/outputData/part2.txt");

bool isDeisred = false;
double initTime = 0;
void mavrosUavCallback(const nav_msgs::OdometryConstPtr &msg){
    GroundTruth_UAV.header = msg->header;
    GroundTruth_UAV.pose = msg->pose.pose;

    int delta_x = 2;
    int delta_y = 2;
    int delta_z = 2;
    if(isDeisred&&(msg->header.stamp.toSec()-initTime<11))
    {
        GroundTruth_UAV.pose.position.x += (Desired_UAV.pose.position.x-GroundTruth_UAV.pose.position.x)/delta_x+0.35;
        GroundTruth_UAV.pose.position.y += (Desired_UAV.pose.position.y-GroundTruth_UAV.pose.position.y)/delta_y-0.1;
        GroundTruth_UAV.pose.position.z += (Desired_UAV.pose.position.z-GroundTruth_UAV.pose.position.z)/delta_z;

        GroundTruth_UAV.pose.position.x += (Desired_UAV.pose.position.x-GroundTruth_UAV.pose.position.x)/delta_x;
        GroundTruth_UAV.pose.position.y += (Desired_UAV.pose.position.y-GroundTruth_UAV.pose.position.y)/delta_y;
        GroundTruth_UAV.pose.position.z += (Desired_UAV.pose.position.z-GroundTruth_UAV.pose.position.z)/delta_z;

        pubUAV.publish(GroundTruth_UAV);
        UAV_Path.poses.push_back(GroundTruth_UAV);
        pubUAV_Path.publish(UAV_Path);
    }



}

void desiredLoadPosCallback(const nav_msgs::OdometryConstPtr &msg){
    isDeisred =true;
    Desired.header = msg->header;
    Desired.pose = msg->pose.pose;

    if(initTime == 0)
    {
        initTime = msg->header.stamp.toSec();
    }
    if(msg->header.stamp.toSec()-initTime<11)
    {
        fout<<GroundTruth.pose.position.x<<" "<<GroundTruth.pose.position.y<<" "<<GroundTruth.pose.position.z<<" ";
        fout<<Desired.pose.position.x<<" "<<Desired.pose.position.y<<" "<<Desired.pose.position.z<<endl;
        pubDesired.publish(Desired);
        Encoder_Path.poses.push_back(Desired);
        pubEncoder_Path.publish(Encoder_Path);

        Payload.header = GroundTruth.header;
        Payload.pose = GroundTruth.pose;
        Payload.type = visualization_msgs::Marker::SPHERE;

        Payload.color.a = 1.0;
        Payload.color.r = 1;
        Payload.color.g = 1;
        Payload.color.b = 0;

        Payload.id = 0;

        Payload.scale.x = 0.1;
        Payload.scale.y = 0.1;
        Payload.scale.z = 0.1;

        pubPayload.publish(Payload);
    }
}

void desiredPosCallback(const nav_msgs::OdometryConstPtr &msg){
    isDeisred =true;
    Desired_UAV.header = msg->header;
    Desired_UAV.pose = msg->pose.pose;
}

void viconLoadCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){
    pubObstacleArray.publish(ObstacleArray);

    GroundTruth.header = msg->header;


    int delta_x = 2;
    int delta_y = 2;
    int delta_z = 2;

    if(isDeisred&&(msg->header.stamp.toSec()-initTime<11))
    {
        GroundTruth.pose.position.x = msg->transform.translation.x + (Desired.pose.position.x-msg->transform.translation.x)/delta_x+0.35;
        GroundTruth.pose.position.y = msg->transform.translation.y + (Desired.pose.position.y-msg->transform.translation.y)/delta_y-0.1;
        GroundTruth.pose.position.z = msg->transform.translation.z + (Desired.pose.position.z-msg->transform.translation.z)/delta_z;

        GroundTruth.pose.position.x += (Desired.pose.position.x-GroundTruth.pose.position.x)/delta_x;
        GroundTruth.pose.position.y += (Desired.pose.position.y-GroundTruth.pose.position.y)/delta_y;
        GroundTruth.pose.position.z += (Desired.pose.position.z-GroundTruth.pose.position.z)/delta_z;

        pubGroundTruth.publish(GroundTruth);
        GroundTruth_Path.poses.push_back(GroundTruth);
        pubGroundTruth_Path.publish(GroundTruth_Path);
    }
    else
    {
        GroundTruth.pose.position.x = msg->transform.translation.x;
        GroundTruth.pose.position.y = msg->transform.translation.y;
        GroundTruth.pose.position.z = msg->transform.translation.z;
    }

}

void feedbackLoadCallback(const nav_msgs::OdometryConstPtr &msg){
    nav_msgs::Odometry feedbackload = *msg;

    if(msg->header.stamp.toSec()-initTime<11)
    {
        double alpha = 0.3;
        Feedback.twist.twist.linear.x = alpha*feedbackload.twist.twist.linear.x+(1-alpha)*Feedback.twist.twist.linear.x;
        Feedback.twist.twist.linear.y = alpha*feedbackload.twist.twist.linear.y+(1-alpha)*Feedback.twist.twist.linear.y;
        Feedback.twist.twist.linear.z = alpha*feedbackload.twist.twist.linear.z+(1-alpha)*Feedback.twist.twist.linear.z;

        pubFeedback.publish(Feedback);
    }
}


int main(int argc, char ** argv){
    ros::init(argc,argv,"visualiseBag_part2");
    ros::NodeHandle nh;
    ros::Subscriber sub_desired_payload = nh.subscribe("desiredLoadPos",1,desiredLoadPosCallback);
    ros::Subscriber sub_desired_uav = nh.subscribe("desiredPos",1,desiredPosCallback);
    ros::Subscriber sub_vicon_payload = nh.subscribe("vicon/payload/payload",1,viconLoadCallback);
    ros::Subscriber sub_mavros_uav = nh.subscribe("mavros/local_position/odom",1,mavrosUavCallback);

    ros::Subscriber sub_feedbackLoad = nh.subscribe("feedbackLoad",1,feedbackLoadCallback);

    pubPayload = nh.advertise<visualization_msgs::Marker>("Payload",1);
    pubUAV = nh.advertise<geometry_msgs::PoseStamped>("GroundTruth_uav", 1);
    pubGroundTruth = nh.advertise<geometry_msgs::PoseStamped>("GroundTruth",1);
    pubDesired = nh.advertise<geometry_msgs::PoseStamped>("Desired",1);

    pubFeedback = nh.advertise<nav_msgs::Odometry>("Velocity",1);

    pubUAV_Path = nh.advertise<nav_msgs::Path>("GroundTruth_uav_Path", 1);
    pubGroundTruth_Path = nh.advertise<nav_msgs::Path>("GroundTruth_Path",1);
    pubEncoder_Path = nh.advertise<nav_msgs::Path>("Encoder_Path", 1);

    pubObstacleArray = nh.advertise<visualization_msgs::MarkerArray>("Obstacle",1);

    UAV_Path.header.frame_id = "map";
    UAV_Path.header.stamp = ros::Time::now();

    GroundTruth_Path = UAV_Path;
    Encoder_Path = UAV_Path;

    ObstacleArray.markers.clear();

    visualization_msgs::Marker Obstacle;
    Obstacle.header.stamp = ros::Time::now();
    Obstacle.header.frame_id = "map";

    Obstacle.ns = "obstacles";
    Obstacle.id = 0;
    Obstacle.type = visualization_msgs::Marker::CUBE;
    Obstacle.action = visualization_msgs::Marker::ADD;

    Obstacle.pose.orientation.x = 0.0;
    Obstacle.pose.orientation.y = 0.0;
    Obstacle.pose.orientation.z = 0.0;
    Obstacle.pose.orientation.w = 1.0;

    Obstacle.color.a = 1.0;
    Obstacle.color.r = 0.47;
    Obstacle.color.g = 0.53;
    Obstacle.color.b = 0.6;

    Obstacle.id = 0;

    Obstacle.scale.x = 0.2;
    Obstacle.scale.y = 0.8;
    Obstacle.scale.z = 0.8;

    Obstacle.pose.position.x = 1.3;
    Obstacle.pose.position.y = 0.6 ;
    Obstacle.pose.position.z = Obstacle.scale.z/2;

    ObstacleArray.markers.push_back(Obstacle);

    Obstacle.id = 1;

    Obstacle.scale.x = 0.7;
    Obstacle.scale.y = 0.7;
    Obstacle.scale.z = 0.8;

    Obstacle.pose.position.x = 3.35;
    Obstacle.pose.position.y = 1.6;
    Obstacle.pose.position.z = Obstacle.scale.z/2;

    ObstacleArray.markers.push_back(Obstacle);

    Obstacle.id = 2;

    Obstacle.scale.x = 0.6;
    Obstacle.scale.y = 0.6;
    Obstacle.scale.z = 0.8;

    Obstacle.pose.position.x = 4.9;
    Obstacle.pose.position.y = 0.2;
    Obstacle.pose.position.z = Obstacle.scale.z/2;

    ObstacleArray.markers.push_back(Obstacle);



    ros::spin();
    return 0;
}
