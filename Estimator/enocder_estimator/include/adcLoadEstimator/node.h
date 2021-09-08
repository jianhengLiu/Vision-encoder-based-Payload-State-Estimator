//
// Created by kevin on 2020/7/12.
//

#ifndef WTR_USB2ROS_NODE_H
#define WTR_USB2ROS_NODE_H
#include "adcLoadEstimator//common_include.h"
namespace wtr{
    class Node{
        ros::NodeHandle nh_;

        ros::Publisher adcPub;
    public:

        Node()
        {
            adcPub = nh_.advertise<geometry_msgs::PoseStamped>("/uav/load/estimation",1);
        }
        void adcPublish(double * data){
            geometry_msgs::PoseStamped pubPose;
            pubPose.pose.position.x = -data[0];
            pubPose.pose.position.y = data[1];
            pubPose.pose.position.z = data[2];
            pubPose.header.stamp = ros::Time::now();
            pubPose.header.frame_id = "body";
            adcPub.publish(pubPose);
        }
    };
}
#endif //WTR_USB2ROS_NODE_H
