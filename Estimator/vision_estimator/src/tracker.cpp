#include "staple_tracker.hpp"

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


using namespace std;
using namespace cv;

double fx = 197.8071807171588;
double fy = 198.0528881923843;

VideoWriter writer;

cv::Point2f refineTracker(cv::Mat input, int times)
{
    int cols = input.cols;
    int rows = input.rows;

    cvtColor(input, input, cv::COLOR_BGR2GRAY);

    for (int i = 0; i < times; ++i)
    {
        cv::pyrUp(input, input);
    }

//    cv::threshold(input, input, input.at<uchar>(input.cols / 2, input.rows / 2) - 10, 255, cv::THRESH_BINARY);

    cv::threshold(input, input, 50, 255, cv::THRESH_BINARY);

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    findContours(input, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    int maxIdx = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i]) > maxArea) {
            maxArea = contourArea(contours[i]);
            maxIdx = i;
        }
        /*if (contourArea(contours[i]) > 400 * pow(4, times))
        {
            cv::RotatedRect returnRect = minAreaRect(contours[i]);
            cv::Point2f returnPoint = returnRect.center;
//            cv::circle(input, returnPoint, 2, cv::Scalar(255), -1);
            returnPoint.x /= pow(2, times);
            returnPoint.y /= pow(2, times);
//            imshow("rectImage", input);
            return returnPoint;
        }*/
    }

    cv::RotatedRect returnRect = minAreaRect(contours[maxIdx]);
    cv::Point2f returnPoint = returnRect.center;
//            cv::circle(input, returnPoint, 2, cv::Scalar(255), -1);
    returnPoint.x /= pow(2, times);
    returnPoint.y /= pow(2, times);
//            imshow("rectImage", input);
    return returnPoint;
//    return cv::Point2f(cols / 2, rows / 2);
}

STAPLE_TRACKER staple;

void trackerStapleInit(cv::Mat input)
{
    namedWindow("STAPLE", cv::WINDOW_AUTOSIZE);
//    cv::Rect roi = selectROI("STAPLE", input);
    cv::Rect roi(279, 169,44 , 43);
    if (roi.width == 0 || roi.height == 0)
    {
        cout << "Failed to get target ROI!" << endl;
        return;
    }

    staple.tracker_staple_initialize(input, roi);
    staple.tracker_staple_train(input, true);
}

bool isTrackerStapleInit = false;
std::vector<cv::Rect_<float>> result_rects;
bool show_visualization = true;
double calculateTime = 0;

Eigen::Vector3d q_sum(0,0,0);
int cnt = 0;
Eigen::Vector3d sigma_sum(0,0,0);
double u_sum=0;
double v_sum = 0;
double sigma_sum_u = 0;
double sigma_sum_v = 0;
void trackerStaple(cv::Mat input)
{
    if (!isTrackerStapleInit)
    {
        trackerStapleInit(input);
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

//    float errorX = (float) (-input.cols) / 2 + targetPoint.x;
//    float errorY = (float) (-input.rows) / 2 + targetPoint.y;
    float errorX = targetPoint.x - (float) (input.cols) / 2;
    float errorY = targetPoint.y - (float) (input.rows) / 2;

    float u = errorX * 1 / fx;
    float v = errorY * 1 / fy;

    u_sum+=u;
    v_sum+=v;



    Eigen::Vector3d q_payload_C(u, v, 1 );
    q_payload_C = q_payload_C.normalized();
    q_sum+=q_payload_C;
    cnt++;
//    cout<<"u_avg:"<<u_sum/cnt<<endl;
//    cout<<"v_avg:"<<v_sum/cnt<<endl;
//    sigma_sum_u+=(u+0.103571)*(u+0.103571);
//    sigma_sum_v+=(v+0.248991)*(v+0.248991);
//    cout<<"u_<sigma:"<<sigma_sum_u/cnt<<endl;
//    cout<<"v_<sigma:"<<sigma_sum_v/cnt<<endl;
//    Eigen::Vector3d q_average = q_sum/cnt;
//    cout<<"q_average"<<endl<<q_average<<endl;
    Eigen::Vector3d q_average(-0.0974537,-0.241383,0.965483);
    sigma_sum+=(q_payload_C-q_average).cwiseProduct(q_payload_C-q_average);//*(q_payload_C-q_average);
    Eigen::Vector3d sigma = sigma_sum/cnt;
    cout<<"sigma"<<endl<<sigma<<endl;
    if (show_visualization)
    {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        cv::putText(input, std::to_string(fps), cv::Point(20, 40), 6, 1,
                    cv::Scalar(0, 255, 255), 2);
//        cv::circle(input, targetPoint, 2, cv::Scalar(255, 0, 0), -1);
        cv::rectangle(input, location, cv::Scalar(0, 128, 255), 2);
        cv::imshow("STAPLE", input);
        writer<<input;

    } else
    {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        std::cout << "fps:" << fps << std::endl;
    }
	cv::waitKey(1);
}

void callbackCamera(const sensor_msgs::ImageConstPtr &floorImage)
{
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(floorImage, "bgr8");
    cv::Mat img_floor = cv_ptr->image;
    /**
     * 0 means flipping around the x-axis
     * positive value (for example, 1) means flipping around y-axis.
     * Negative value (for example, -1) means flipping around both axes.
     */
    flip(img_floor, img_floor, 0);
    trackerStaple(img_floor);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subFloorCamera = it.subscribe("/undistortFisheye", 1, callbackCamera);

    writer = VideoWriter("/home/chrisliu/ROSws/not_now/ICRA2021Test_ws/src/STAPLE/IROS2021_04.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, Size(640, 480));

    while (ros::ok())
    {
        ros::spinOnce();
    }

    ros::waitForShutdown();
    return 0;
}


