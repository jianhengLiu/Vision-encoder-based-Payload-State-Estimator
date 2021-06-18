//
// Created by chrisliu on 2020/10/21.
//
#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;

VideoWriter writer;

int main(int argc, char **argv)
{
    writer = VideoWriter("/home/chrisliu/NewDisk/ROSws/ICRA2021Test_ws/src/img_extract/extract_video/ICRA2021part1_01.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, Size(640, 480));

    ros::init(argc, argv, "fisheye_undistort_node");
    ros::NodeHandle nh;
//    ros::Rate rate(30);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_undistort_fisheye = it.advertise("undistortFisheye",1);

    cv::Matx33d intrinsic_matrix;
    intrinsic_matrix<<197.8071807171588, 0, 317.3925061384536,
    0, 198.0528881923843, 247.5847385439813,
    0, 0, 1;
//    200.1230529821021, 0, 319.0261836498302,
//    0, 200.050750054437, 218.9617650999124,
//    0, 0, 1;
    cv::Vec4d distortion_coeffs;
    distortion_coeffs<<0.588975, 0.228545, -0.516537, 0.23191;//0.664207, 0.0574451, -0.335926, 0.160933;
    Mat R = Mat::eye(3,3,CV_32F);
    Size image_size(640,480);
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);

    VideoCapture inputVideo = VideoCapture(2);
    if (!inputVideo.isOpened())
    {
        inputVideo = VideoCapture(0);
        if (!inputVideo.isOpened()){
            cout << "Could not open the input video " << endl;
            return -1;
        }
    }
    Mat frame;
    Mat frameUndistort;
    while (ros::ok())
    {
        inputVideo >> frame;

        cv::remap(frame,frameUndistort,mapx, mapy, INTER_LINEAR);

        pub_undistort_fisheye.publish(cv_bridge::CvImage(std_msgs::Header(),"bgr8",frameUndistort).toImageMsg());
//        writer<<frame1    Undistort;

//        rate.sleep();
//            imshow("undistortImg",frameUndistort);
//    char key = waitKey(1);
//    if (key == 'q' || key == 'Q')
//    {
//        imwrite("/home/chrisliu/NewDisk/ROSws/ICRA2021Test_ws/src/STAPLE/template_imgs/template_payload1.jpg", frameUndistort);
//    }
    }
}
