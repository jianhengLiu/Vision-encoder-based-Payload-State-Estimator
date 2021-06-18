//
// Created by chrisliu on 2020/10/14.
//

//
// Created by chrisliu on 2020/4/9.
//
#include "ros/ros.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

VideoWriter writer("/home/chrisliu/NewDisk/ROSws/img_ws/src/img_extract/extract_video/VideoTest.mp4v", CV_FOURCC('D', 'I', 'V', 'X'), 30.0, Size(640, 480));

int n_imgs = 0;
double now = 0;
double past = 0;
void callbackCamera(const sensor_msgs::ImageConstPtr &Image)
{
    now = ros::Time::now().toSec();
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(Image, "bgr8");
    cv::Mat img = cv_ptr->image;
    writer<<img;
//    if(now-past>0.5&&n_imgs<150)
//    {
//
//        imwrite("/home/chrisliu/NewDisk/ROSws/img_ws/src/img_extract/extract_imgs/img_" + to_string(n_imgs) + ".jpg",img);
//        n_imgs++;
//        past = now;
//    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subFloorCamera = it.subscribe("/realsense/image_raw", 1, callbackCamera);


    while (ros::ok())
    {
        ros::spinOnce();
    }

    ros::waitForShutdown();
    return 0;
}