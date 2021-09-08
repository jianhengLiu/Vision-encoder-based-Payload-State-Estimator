#include "kf/common_include.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using namespace std;
queue<MeasurementPackage> msgQueue;

ros::Time lastTime;
void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr &poseMsg){

//            x_[2] = sqrt(length*length/(1+tan(theta1)*tan(theta1) + tan(theta2)* tan(theta2)));
//            x_[0] = x_[2]*tan(theta2);
//            x_[1] = x_[2]*tan(theta1);
    if(msgQueue.size() <2){
        MeasurementPackage mearNow;
        mearNow.sensor_type_ = MeasurementPackage::LASER;
        mearNow.raw_measurements_ = VectorXd(2);
        float theta1 = (float)poseMsg->data[0];
        float theta2 = (float)poseMsg->data[1];
        float length =0.83;
        float z = sqrt(length*length/(1+tan(theta1)*tan(theta1) + tan(theta2)* tan(theta2)));
        float xx = z*tan(theta2);
        float yy = z*tan(theta1);
        mearNow.raw_measurements_ << xx, yy;
        mearNow.timestamp_ = ros::Time::now().toNSec();
        msgQueue.push(mearNow);
    }


}
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

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "kf");

    ros::NodeHandle nh;

    ros::Subscriber encoderSub = nh.subscribe<std_msgs::Float32MultiArray>("/uav/load/estimation/encoder/angle",1,encoderCallback);
//    ros::Subscriber visionSub = nh.subscribe<geometry_msgs::PoseStamped>("/uav/load/estimation/vision",1,visionCallback);
    ros::Publisher fusionPub = nh.advertise<geometry_msgs::PoseStamped>("/uav/load/estimation/fusion",1);
    geometry_msgs::PoseStamped pubFusion;
    // Create a Fusion EKF instance
    UKF ukf;

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    ros::Rate loopRate(100);
    while(ros::ok()) {

        // start filtering from the second frame (the speed is unknown in the first
        // frame)
//        if(measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) continue;
        if(!msgQueue.empty()){
            ukf.ProcessMeasurement(msgQueue.front());

            double p_x = ukf.x_(0); ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
            double p_y = ukf.x_(1);
            printf("Estimated load position (%.3lf, %.3lf)\n",p_x,p_y);
            pubFusion.header.stamp = ros::Time::now();
            pubFusion.header.frame_id = "body";
            pubFusion.pose.position.x = p_x;
            pubFusion.pose.position.y = p_y;
            fusionPub.publish(pubFusion);
            msgQueue.pop();

        }
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}