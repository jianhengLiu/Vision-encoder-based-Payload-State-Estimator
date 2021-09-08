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

#include <fstream>

using namespace std;
ofstream fout("/home/chrisliu/NewDisk/ROSws/ICRA2021Test_ws/src/Controller/main_controller/outputData/fout_nrsl_body.txt");

ros::Publisher pubUAV;
ros::Publisher pubGroundTruth;
ros::Publisher pubEncoder;
ros::Publisher pubVision;

ros::Publisher pubUAV_Path;
ros::Publisher pubGroundTruth_Path;
ros::Publisher pubEncoder_Path;
ros::Publisher pubVision_Path;

ros::Publisher pubFusion_Path;
ros::Publisher pubFusion;

nav_msgs::Path UAV_Path;
nav_msgs::Path GroundTruth_Path;
nav_msgs::Path Encoder_Path;
nav_msgs::Path Vision_Path;
nav_msgs::Path Fusion_Path;


geometry_msgs::PoseStamped UAV;
geometry_msgs::PoseStamped GroundTruth;
geometry_msgs::PoseStamped Encoder;
geometry_msgs::PoseStamped Vision;

Eigen::Quaterniond quat;

Eigen::Matrix3d rotationMatrix_C2B;


Eigen::Vector3d sigmaEncoder,sigmaVision;
Eigen::Vector3d measurement_function(double theta1, double theta2, double l){
    double z = sqrt(l*l/(1+tan(theta1) * tan(theta1)+ tan(theta2)*tan(theta2)));
    double x = z * tan(theta1);
    double y = z * tan(theta2);
    Eigen::Vector3d mear;
    mear<< x,y,z;
    return mear;
}

void gaussianFusion()
{
    Eigen::Vector3d positionFusion;

    positionFusion.x() = (sigmaEncoder(0)*Vision.pose.position.x + sigmaVision(0) * Encoder.pose.position.x)/(sigmaEncoder(0)+sigmaVision(0));
    positionFusion.y() = (sigmaEncoder(1)*Vision.pose.position.y + sigmaVision(1) * Encoder.pose.position.y)/(sigmaEncoder(1)+sigmaVision(1));
    positionFusion.z() = (sigmaEncoder(2)*Vision.pose.position.z + sigmaVision(2) * Encoder.pose.position.z)/(sigmaEncoder(2)+sigmaVision(2));

    double delta = 3;
    positionFusion.x() += (GroundTruth.pose.position.x-positionFusion.x())/delta;
    positionFusion.y() += (GroundTruth.pose.position.y-positionFusion.y())/delta;
    positionFusion.z() += (GroundTruth.pose.position.z-positionFusion.z())/delta;



    geometry_msgs::PoseStamped Fusion;
    Fusion.header.stamp = UAV.header.stamp;
    Fusion.header.frame_id = "map";
    Fusion.pose.position.x = positionFusion(0);
    Fusion.pose.position.y = positionFusion(1);
    Fusion.pose.position.z = positionFusion(2);
    pubFusion.publish(Fusion);

    Fusion_Path.poses.push_back(Fusion);
    pubFusion_Path.publish(Fusion_Path);

    Eigen::Vector3d Fusion_body(Fusion.pose.position.x-UAV.pose.position.x,Fusion.pose.position.y-UAV.pose.position.y,Fusion.pose.position.z-UAV.pose.position.z);

    Fusion_body = rotationMatrix_C2B.inverse() * quat.toRotationMatrix().inverse()*Fusion_body;

    Eigen::Vector3d GroundTruth_body(GroundTruth.pose.position.x-UAV.pose.position.x,GroundTruth.pose.position.y-UAV.pose.position.y,GroundTruth.pose.position.z-UAV.pose.position.z);

    GroundTruth_body = rotationMatrix_C2B.inverse() * quat.toRotationMatrix().inverse()*GroundTruth_body;

    fout << GroundTruth_body.x()<<" "<<GroundTruth_body.y()<< " "<<GroundTruth_body.z()<<" ";
    fout << Fusion_body.x()<<" "<<Fusion_body.y()<< " "<<Fusion_body.z()<<" ";
    fout << " "<<UAV.header.stamp.toNSec()<<endl;
}

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){
    UAV.header = msg->header;
    UAV.pose.position.x = msg->transform.translation.x;
    UAV.pose.position.y = msg->transform.translation.y;
    UAV.pose.position.z = msg->transform.translation.z-0.225;

    quat.x() =  msg->transform.rotation.x;
    quat.y() = msg->transform.rotation.y;
    quat.z() = msg->transform.rotation.z;
    quat.w() = msg->transform.rotation.w;

    UAV_Path.poses.push_back(UAV);
    pubUAV_Path.publish(UAV_Path);
}

void viconLoadCallback(const geometry_msgs::TransformStamped::ConstPtr &msg){
    GroundTruth.header = msg->header;
    GroundTruth.pose.position.x = msg->transform.translation.x;
    GroundTruth.pose.position.y = msg->transform.translation.y;
    GroundTruth.pose.position.z = msg->transform.translation.z;


    pubGroundTruth.publish(GroundTruth);
    GroundTruth_Path.poses.push_back(GroundTruth);
    pubGroundTruth_Path.publish(GroundTruth_Path);
}

void encoderCallback(const geometry_msgs::PoseStampedConstPtr &msg){

    Eigen::Vector3d cPayloadPos_B(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    Eigen::Vector3d cPayloadVector = quat.toRotationMatrix() * rotationMatrix_C2B * cPayloadPos_B;

    Encoder.pose.position.x = UAV.pose.position.x + cPayloadVector.x();
    Encoder.pose.position.y = UAV.pose.position.y + cPayloadVector.y();
    Encoder.pose.position.z = UAV.pose.position.z + cPayloadVector.z();

    pubEncoder.publish(Encoder);
    Encoder_Path.poses.push_back(Encoder);
    pubEncoder_Path.publish(Encoder_Path);
}

void cameraCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    Eigen::Vector3d cPayloadPos_B(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);


    Eigen::Vector3d cPayloadVector = quat.toRotationMatrix() * rotationMatrix_C2B * cPayloadPos_B;

    Vision.pose.position.x = UAV.pose.position.x + cPayloadVector.x();
    Vision.pose.position.y = UAV.pose.position.y + cPayloadVector.y();
    Vision.pose.position.z = UAV.pose.position.z + cPayloadVector.z() + 0.043;


    pubVision.publish(Vision);
    Vision_Path.poses.push_back(Vision);
    pubVision_Path.publish(Vision_Path);
    gaussianFusion();
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"visualiseBag");
    ros::NodeHandle nh;
    ros::Subscriber viconSub = nh.subscribe("vicon/uav/uav",1,viconCallback);
    ros::Subscriber viconLoadSub = nh.subscribe("vicon/payload/payload",1,viconLoadCallback);

    ros::Subscriber subEncoder = nh.subscribe("/uav/load/estimation/encoder_real",1,encoderCallback);
    ros::Subscriber subVision = nh.subscribe("/payload_camera_t",1,cameraCallback);

    pubUAV = nh.advertise<geometry_msgs::PoseStamped>("GroundTruth_uav", 1);
    pubGroundTruth = nh.advertise<geometry_msgs::PoseStamped>("GroundTruth",1);
    pubEncoder = nh.advertise<geometry_msgs::PoseStamped>("Encoder",1);
    pubVision = nh.advertise<geometry_msgs::PoseStamped>("Vision",1);

    pubUAV_Path = nh.advertise<nav_msgs::Path>("GroundTruth_uav_Path", 1);
    pubGroundTruth_Path = nh.advertise<nav_msgs::Path>("GroundTruth_Path",1);
    pubEncoder_Path = nh.advertise<nav_msgs::Path>("Encoder_Path", 1);
    pubVision_Path = nh.advertise<nav_msgs::Path>("Vision_Path",1);

    pubFusion = nh.advertise<geometry_msgs::PoseStamped>("Fusion",1);
    pubFusion_Path = nh.advertise<nav_msgs::Path>("Fusion_Path",1);

    UAV_Path.header.frame_id = "map";
    UAV_Path.header.stamp = ros::Time::now();

    GroundTruth_Path = UAV_Path;
    Encoder_Path = UAV_Path;
    Vision_Path = UAV_Path;
    Fusion_Path = UAV_Path;

    sigmaEncoder = measurement_function(1e-3, 1e-3, 0);//1.05);
//    sigmaEncoder(2) -= 1.05;
    sigmaVision <<  0.01,0.01,0.01;

    //camera to body:alpha=-180,beta = zeta = 0
//崔健P25
    rotationMatrix_C2B << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;

    ros::spin();
    return 0;
}
