//
// Created by kevin on 2020/8/14.
//

//
// Created by kevin on 2020/8/14.
//

#include "kf/common_include.h"
#include "kf/ekf.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
queue<MeasurementPackage> msgQueue;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
        cout<<"the input is not legal!!!"<<endl;
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse = rmse + residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;

}

void encoderCallback(const std_msgs::Float32MultiArrayConstPtr &poseMsg){
    //msgQueue.push();
    if(msgQueue.size()<3){
    MeasurementPackage mearNow;
    mearNow.sensor_type_ = MeasurementPackage::ENCODER;
    mearNow.raw_measurements_ = VectorXd(2);
    mearNow.raw_measurements_ << poseMsg->data[0], poseMsg->data[1];
    mearNow.timestamp_ = ros::Time::now().toNSec();
    msgQueue.push(mearNow);
    }

}
void visionCallback(const geometry_msgs::PoseStampedConstPtr &visionMsg){

    MeasurementPackage mearNow;
    mearNow.sensor_type_ = MeasurementPackage::VISION;
    mearNow.raw_measurements_ = VectorXd(3);
    mearNow.raw_measurements_ << (float)visionMsg->pose.position.x, (float)visionMsg->pose.position.y,(float)visionMsg->pose.position.z;
    mearNow.timestamp_ = ros::Time::now().toNSec();
    msgQueue.push(mearNow);


}

int main(int argc, char* argv[]) {
    ///* Subscribe Data from ROS
    ros::init(argc, argv, "kf");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;

    ros::Subscriber encoderSub = nh.subscribe<std_msgs::Float32MultiArray>("/uav/load/estimation/encoder/angle",1,encoderCallback);
    ros::Subscriber visionSub = nh.subscribe<geometry_msgs::PoseStamped>("/uav/load/estimation/vision",1,visionCallback);
    ros::Publisher fusionPub = nh.advertise<geometry_msgs::PoseStamped>("/uav/load/estimation/fusion",1);
    geometry_msgs::PoseStamped pubFusion;
    // Create a Fusion EKF instance
    EKF ekf;

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    ros::Rate loopRate(100);

    while(ros::ok()) {
        // start filtering from the second frame (the speed is unknown in the first
        // frame)
        if(!msgQueue.empty()){
//            if(msgQueue.front().sensor_type_ == MeasurementPackage::VISION)
//            {
//                msgQueue.pop();
//                continue;
//            }
            ekf.ProcessMeasurement(msgQueue.front());

            double theta1 = ekf.x_(0); ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
            double theta2 = ekf.x_(1);

            float l = 1.05;
            float p_z = sqrt(l*l/(1+tan(theta1)*tan(theta1) + tan(theta2)* tan(theta2)));
            float p_x = p_z * tan(theta1);
            float p_y = p_z * tan(theta2);

            pubFusion.header.stamp = ros::Time::now();
            pubFusion.header.frame_id = "body";
            pubFusion.pose.position.x = p_x;
            pubFusion.pose.position.y = p_y;
            pubFusion.pose.position.z = p_z;


            fusionPub.publish(pubFusion);
            msgQueue.pop();
        }
        loopRate.sleep();
        ros::spinOnce();
    }

    // compute the accuracy (RMSE)
    cout << "Accuracy - RMSE:" << endl << CalculateRMSE(estimations, ground_truth) << endl;



    return 0;
}