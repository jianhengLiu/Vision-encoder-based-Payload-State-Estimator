#include "staple_tracker.hpp"

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Eigen>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


using namespace std;
using namespace cv;

ros::Publisher pub_payload_pos_C;
geometry_msgs::PoseStamped payload_pos_C;

double fx = 200.1230529821021;
double fy = 200.050750054437;
double length = 0.57;//0.8;//0.72;
double error_y = 0.082;
double error_x = 0.000;
double error_z = 0.025;//0.034;
double error_length = sqrt(error_x*error_x+error_y*error_y+error_z*error_z);
double error_length2 = error_length*error_length;
double error_rad = atan(error_z/sqrt(error_x*error_x+error_y*error_y));

cv::Point2f refineTracker(cv::Mat input, int times)
{
    int cols = input.cols;
    int rows = input.rows;

    cvtColor(input, input, cv::COLOR_BGR2GRAY);

    for (int i = 0; i < times; ++i)
    {
        cv::pyrUp(input, input);
    }

    cv::threshold(input, input, input.at<uchar>(input.cols / 2, input.rows / 2) - 10, 255, cv::THRESH_BINARY);

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    findContours(input, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i]) > 400 * pow(4, times))
        {
            cv::RotatedRect returnRect = minAreaRect(contours[i]);
            cv::Point2f returnPoint = returnRect.center;
            cv::circle(input, returnPoint, 2, cv::Scalar(255), -1);
            returnPoint.x /= pow(2, times);
            returnPoint.y /= pow(2, times);
//            imshow("rectImage", input);
            return returnPoint;
        }
    }
    return cv::Point2f(cols / 2, rows / 2);
}

STAPLE_TRACKER staple;

void trackerStapleInit()
{
    payload_pos_C.header.frame_id = "map";
    payload_pos_C.pose.orientation.x = 0;
    payload_pos_C.pose.orientation.y = 0;
    payload_pos_C.pose.orientation.z = 0;
    payload_pos_C.pose.orientation.w = 1;

    namedWindow("STAPLE", cv::WINDOW_AUTOSIZE);
    Mat template_img = imread("/home/chrisliu/ROSws/not_now/ICRA2021Test_ws/src/STAPLE/template_imgs/template_payload1.jpg");

//    cv::Rect roi(301, 228,47 , 50);//(305, 234,40 , 40);
    cv::Rect roi = selectROI("STAPLE", template_img);
    cout<<roi<<endl;
    if (roi.width == 0 || roi.height == 0)
    {
        cout << "Failed to get target ROI!" << endl;
        return;
    }

    staple.tracker_staple_initialize(template_img, roi);
    staple.tracker_staple_train(template_img, true);
}

bool isTrackerStapleInit = false;
std::vector<cv::Rect_<float>> result_rects;
bool show_visualization = true;
double calculateTime = 0;

void trackerStaple(cv::Mat input)
{
    if (!isTrackerStapleInit)
    {
        trackerStapleInit();
        isTrackerStapleInit = true;
    }

    int64 tic = cv::getTickCount();

    cv::Rect_<float> location = staple.tracker_staple_update(input);
    staple.tracker_staple_train(input, false);
    result_rects.push_back(location);

    cv::Point2f targetPoint = location.tl();
    if (location.x > 0 && location.y > 0 && location.x + location.width < input.cols &&
        location.y + location.height < input.rows)
    {
        cv::Point2f returnPoint = refineTracker(input(location), 4);
        targetPoint += returnPoint;
    } else
    {
        targetPoint.x += location.width / 2;
        targetPoint.y += location.height / 2;
    }

    int64 toc = cv::getTickCount() - tic;
    calculateTime += toc;

    float errorX = targetPoint.x - (float) (input.cols) / 2;
    float errorY = targetPoint.y - (float) (input.rows) / 2;

    float u = errorX * 1 / fx;
    float v = errorY * 1 / fy;
    float det = sqrt(pow(u, 2) + pow(v, 2) + 1);

    Eigen::Vector3d q_payload_C = Eigen::Vector3d(u, v, 1 );
    Eigen::Vector3d e3(0,0,1);
    double theta = 3.1415926535/2-error_rad - acos(q_payload_C.dot(e3)/q_payload_C.norm());
    double length_hat = error_length*cos(theta)+sqrt(error_length2*cos(theta)*cos(theta)-error_length2+length*length);

    payload_pos_C.header.stamp = ros::Time::now();
    payload_pos_C.pose.position.x = -v/ det * length_hat+error_y;//u/ det * length_hat+error_x;
    payload_pos_C.pose.position.y = u/ det * length_hat+error_x;//v/ det * length_hat+error_y;
    payload_pos_C.pose.position.z = 1/ det * length_hat+error_z;
    pub_payload_pos_C.publish(payload_pos_C);

    if (show_visualization)
    {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        cv::putText(input, std::to_string(fps), cv::Point(20, 40), 6, 1,
                    cv::Scalar(0, 255, 255), 2);
        cv::circle(input, targetPoint, 2, cv::Scalar(255, 0, 0), -1);
        cv::rectangle(input, location, cv::Scalar(0, 128, 255), 2);
        cv::imshow("STAPLE", input);

    } else
    {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        std::cout << "fps:" << fps << std::endl;
    }
    cv::waitKey(1);
}

void callbackCamera(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
    cv::Mat img = cv_ptr->image;
    /**
     * 0 means flipping around the x-axis
     * positive value (for example, 1) means flipping around y-axis.
     * Negative value (for example, -1) means flipping around both axes.
     */
//    flip(img, img, 0);
    trackerStaple(img);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(200);

    pub_payload_pos_C = nh.advertise<geometry_msgs::PoseStamped>("/payload_camera",1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subFloorCamera = it.subscribe("undistortFisheye", 1, callbackCamera);


    while (ros::ok())
    {
        ros::spinOnce();
    }

    ros::waitForShutdown();
    return 0;
}


