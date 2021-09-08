//
// Created by kevin on 2020/7/20.
//

#include <iostream>
#include <ros/ros.h>
//msgs type and conversion
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>

using namespace std;
using namespace cv;

double fx=150.0;
double fy=150.0;
double cx= 261.5;
double cy=149.5;
double b= -6.36812225e-02;
Mat Color, inputImg;
ros::Publisher pcl_pub;
ros::Publisher obj_pub;
ros::Publisher load_pub;
geometry_msgs::PoseStamped pose;
void img_cb(const sensor_msgs::ImageConstPtr& msg)
{
//    cout<<"Get Image"<<endl;
    cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    inputImg = cv_ptr->image;
}

void putText(Mat image, int pos_x, int pos_y, float x,float y,float z){
    char text[50];
    sprintf(text,"(%.3f,%.3f,%.3f)" , x,y,z);
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale =0.4;
    int thickness = 1;

    cv::Point origin;
    origin.x = pos_x-50;
    origin.y = pos_y-20 ;

    cv::putText(image, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
}

void pcl_call_back(const sensor_msgs::PointCloud2ConstPtr& input)
{
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    double min_z = 100;
    float x = 0;
    float y = 0;
    float z = 0;
    float dis = 0;
    float dis2d = 0;

    pcl::PointCloud<pcl::PointXYZRGB> inputCloud;
    pcl::fromROSMsg (*input, inputCloud);//cloud is the output
    pcl::PointCloud<pcl::PointXYZRGB> outputCloud;
    min_z = 10;
    for(int i =0;i<inputCloud.points.size();++i)
    {

        x = inputCloud.points[i].x;//+1;
        y = inputCloud.points[i].y;//+0.2;//
        z = inputCloud.points[i].z;
        dis = sqrt(x*x+y*y+z*z);
        dis2d = sqrt(x*x+y*y);
        if(dis>1.1||dis<1.0){//||z>-0.3 || z < -0.5){
            continue;
        }
        else{
            inputCloud.points[i].x = x;
            inputCloud.points[i].y = y;//+0.2;//
            inputCloud.points[i].z = z;
            outputCloud.points.push_back(inputCloud.points[i]);
            sum_x += z;
            sum_y += x;
            sum_z += y;
	    if(min_z>z)min_z = z;
        }
    }
    if(!outputCloud.points.empty())
    {
        sum_x /= outputCloud.points.size();
        sum_y /= outputCloud.points.size();
        sum_z /= outputCloud.points.size();

        int u = (int)(-sum_x*fx+cx);
        int v = (int)(sum_y*fy+cy);

        if(!inputImg.empty())
        {

            double length = sqrt(sum_x*sum_x+sum_y*sum_y+min_z*min_z);
            pose.pose.position.x =sum_y+0.033 - 0.019;
            pose.pose.position.y =  sum_z +0.0079 ;
            pose.pose.position.z=  sum_x;
            printf("Estimated position: (%.3f,%.3f,%.3f), Length:%f \n", sum_y,sum_z,sum_x,length);
//            cvtColor(inputImg,Color,CV_GRAY2BGR);
//            circle(Color,Point(u,v),6,Scalar(0, 255, 255),3);
//            putText(Color, u , v , sum_x , sum_y , min_z);
//            imshow("Img", Color);
//            waitKey(1);
        }
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id =  "body";
        Eigen::Vector3d obj_point(sum_x,sum_y,sum_z);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(outputCloud,output);
        output.header.frame_id = "camera_depth_optical_frame";
        pcl_pub.publish(output);
        load_pub.publish(pose);
    }
}

int main(int argc,char** argv)
{
// Initialize ROS
    ros::init (argc, argv, "pcl_2_pcd");
    ros::NodeHandle nh;

// Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, pcl_call_back);
    ros::Subscriber sub_img = nh.subscribe("/camera/color/image_raw",1,img_cb);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcl",1);
    load_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav/load/estimation/vision", 1);
    ros::spin();
}
