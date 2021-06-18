//
// Created by chrisliu on 2020/10/21.
//
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;
int main()
{
    cv::Matx33d intrinsic_matrix;
    intrinsic_matrix<<200.1230529821021, 0, 319.0261836498302,
    0, 200.050750054437, 218.9617650999124,
    0, 0, 1;
    cv::Vec4d distortion_coeffs;
    distortion_coeffs<<0.664207, 0.0574451, -0.335926, 0.160933;
    Mat R = Mat::eye(3,3,CV_32F);
    Size image_size(640,480);
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);

    VideoCapture inputVideo(4);
    if (!inputVideo.isOpened())
    {
        cout << "Could not open the input video " << endl;
        return -1;
    }
    Mat frame;
    Mat frameCalibration;
    while (1)
    {
        inputVideo >> frame;

        cv::remap(frame,frameCalibration,mapx, mapy, INTER_LINEAR);

        imshow("undistortImg",frameCalibration);
        waitKey(1);
    }
}