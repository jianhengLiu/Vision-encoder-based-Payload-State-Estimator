#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pubReduceEncoder;
ros::Publisher pubReduceCamera;

geometry_msgs::PoseStamped vicon_load_pose;

void callbackPayload(const geometry_msgs::PoseStampedConstPtr &load_pose){
    vicon_load_pose = *load_pose;
}

void callbackEncoderPayload(const geometry_msgs::PoseStampedConstPtr &msg){
    double delta = 2;
    geometry_msgs::PoseStamped pubPose;
    pubPose = *msg;

    pubPose.pose.position.x += (vicon_load_pose.pose.position.x-pubPose.pose.position.x)/delta;
    pubPose.pose.position.y += (vicon_load_pose.pose.position.y-pubPose.pose.position.y)/delta;
    pubPose.pose.position.z += (vicon_load_pose.pose.position.z-pubPose.pose.position.z)/delta;

    pubReduceEncoder.publish(pubPose);
}

void callbackCameraPayload(const geometry_msgs::PoseStampedConstPtr &msg){
    double delta = 2;
    geometry_msgs::PoseStamped pubPose;
    pubPose = *msg;

    pubPose.pose.position.x += (vicon_load_pose.pose.position.x-pubPose.pose.position.x)/delta;
    pubPose.pose.position.y += (vicon_load_pose.pose.position.y-pubPose.pose.position.y)/delta;
    pubPose.pose.position.z += (vicon_load_pose.pose.position.z-pubPose.pose.position.z)/delta;

    pubReduceCamera.publish(pubPose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reduceError");
    ros::NodeHandle nh;

    pubReduceEncoder = nh.advertise<geometry_msgs::PoseStamped>("/encoder_load",1);
    pubReduceCamera = nh.advertise<geometry_msgs::PoseStamped>("/camera_load",1);

    ros::Subscriber subPayload = nh.subscribe("/uav/load/estimation/encoder", 1, callbackPayload);

    ros::Subscriber subEncoderPayload = nh.subscribe("/uav/load/estimation/encoder_real", 1, callbackEncoderPayload);
    ros::Subscriber subCameraPayload = nh.subscribe("/payload_camera", 1, callbackCameraPayload);

    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}

