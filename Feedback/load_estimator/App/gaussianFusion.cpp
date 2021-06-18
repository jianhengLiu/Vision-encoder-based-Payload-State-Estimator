//
// Created by kevin on 2020/8/16.
//


#include "kf/common_include.h"
#include "kf/ekf.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
queue<MeasurementPackage> visionQueue;
queue<MeasurementPackage> encoderQueue;
Eigen::Vector3d sigmaEncoder,sigmaVision;
ros::Publisher fusionPub ;


Eigen::Vector3d measurement_function(double theta1, double theta2, double l){
    double z = sqrt(l*l/(1+tan(theta1) * tan(theta1)+ tan(theta2)*tan(theta2)));
    double x = z * tan(theta1);
    double y = z * tan(theta2);
    Eigen::Vector3d mear;
    mear<< x,y,z;
    return mear;
}

Eigen::Vector3d mapA2P;
void encoderCallback(const std_msgs::Float32MultiArrayConstPtr &poseMsg){
    //msgQueue.push();

        MeasurementPackage mearNow;
        mearNow.sensor_type_ = MeasurementPackage::ENCODER;
        mapA2P = measurement_function( poseMsg->data[0], poseMsg->data[1], poseMsg->data[2]);
        mearNow.raw_measurements_ = VectorXd(3);
        mearNow.raw_measurements_ << mapA2P(0),mapA2P(1),mapA2P(2);
        mearNow.timestamp_ = ros::Time::now().toNSec();
        encoderQueue.push(mearNow);
        if(encoderQueue.size()>2)
            encoderQueue.pop();


}


void visionCallback(const geometry_msgs::PoseStampedConstPtr &visionMsg){


    MeasurementPackage mearNow;
    mearNow.sensor_type_ = MeasurementPackage::VISION;
    mearNow.raw_measurements_ = VectorXd(3);
    mearNow.raw_measurements_ << visionMsg->pose.position.x, visionMsg->pose.position.y,visionMsg->pose.position.z;
    mearNow.timestamp_ = ros::Time::now().toNSec();
    visionQueue.push(mearNow);
    if(visionQueue.size()>2)
        visionQueue.pop();
}


void gaussianFusion()
{
    if(visionQueue.empty() || encoderQueue.empty())
        return;
    Eigen::Vector3d positionFusion;
    for(int i = 0 ; i < 3 ; i ++ ){
        positionFusion(i) = (sigmaEncoder(i)*visionQueue.back().raw_measurements_(i) + sigmaVision(i) * encoderQueue.back().raw_measurements_(i))/(sigmaEncoder(i)+sigmaVision(i));
    }
    geometry_msgs::PoseStamped pubFusion;
    pubFusion.header.stamp = ros::Time::now();
    pubFusion.header.frame_id = "body";
    pubFusion.pose.position.x = positionFusion(0);
    pubFusion.pose.position.y = positionFusion(1);
    pubFusion.pose.position.z = positionFusion(2);
    fusionPub.publish(pubFusion);

    visionQueue.pop();
    encoderQueue.pop();

}


int main(int argc, char* argv[]) {
    ///* Subscribe Data from ROS
    ros::init(argc, argv, "kf");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;

    ros::Subscriber encoderSub = nh.subscribe<std_msgs::Float32MultiArray>("/uav/load/estimation/encoder/angle",1,encoderCallback);
    ros::Subscriber visionSub = nh.subscribe<geometry_msgs::PoseStamped>("/uav/load/estimation/vision",1,visionCallback);
    fusionPub = nh.advertise<geometry_msgs::PoseStamped>("/uav/load/estimation/fusion",1);


    sigmaEncoder = measurement_function(1e-3, 1e-3, 1.05);
    sigmaEncoder(2) -= 1.05;
    sigmaVision <<  0.01,0.01,0.01;
    // used to compute the RMSE later

    ros::Rate loopRate(1000);

    while(ros::ok()) {
        gaussianFusion();
        loopRate.sleep();
        ros::spinOnce();
    }


    return 0;
}