//
// Created by kevin on 2020/8/5.
//

#ifndef LOADCONTROLLER_NODE_H
#define LOADCONTROLLER_NODE_H

#include "loadController/common_include.h"

namespace wtr{

    class Node{

        ros::NodeHandle nh;

        ros::Publisher pubRotorRevs;
        ros::Publisher pubAttComm;
        ros::Publisher local_pos_pub;
        ros::Publisher pubDesiredPos;
        ros::Publisher pubFeedbackLoad;
        ros::Publisher pubLoadDesieredPos;

        ros::Subscriber subPayload;
        ros::Subscriber subTarget;
        ros::Subscriber subOdometry;
        ros::Subscriber subIMU;
        ros::Subscriber subTrajectory;
        ros::Subscriber subPX4State;
        ros::Subscriber subPixelError;
        ros::Subscriber subEncoder;

        ros::Subscriber subEncoderPayload;
        ros::Subscriber subCameraPayload;

        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;

        mavros_msgs::AttitudeTarget AttTar;
        mavros_msgs::State current_state;
    private:
        void callbackOdometry(const nav_msgs::OdometryConstPtr &odom)
        {
            controller.updateOdom(odom);
        }

        void callbackImu(const sensor_msgs::ImuConstPtr &imu)
        {
            controller.updateImu(imu);
        }

        void callbackPayload(const geometry_msgs::PoseStampedConstPtr &load_pose){
            controller.updatePayload(load_pose);
        }

        void state_cb(const mavros_msgs::StateConstPtr& msg){
            current_state = *msg;
        }

        void callbackEncoder(const geometry_msgs::TransformStamped::ConstPtr &msg){

            Eigen::Quaterniond quat;
            quat.x() = msg->transform.rotation.x;
            quat.y() = msg->transform.rotation.y;
            quat.z() = msg->transform.rotation.z;
            quat.w() = msg->transform.rotation.w;

            controller.feedback.cBodyRotationMatrix = quat.toRotationMatrix();
            controller.feedback.cBodyPosition.x() = msg->transform.translation.x;
            controller.feedback.cBodyPosition.y() = msg->transform.translation.y;
            controller.feedback.cBodyPosition.z() = msg->transform.translation.z;
        }

    public:
        ros::Publisher pubDrop;
        PayloadController controller;

        Vec3 targetPosition;
        Eigen::Quaterniond targetQuaternion;

        ros::Time last_request;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;

        Node(){
            // 全体置零
            memset(&controller, 0 , sizeof(controller));

            pubAttComm = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
            local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

            pubDesiredPos = nh.advertise<nav_msgs::Odometry>("/desiredPos",1);
            pubFeedbackLoad = nh.advertise<nav_msgs::Odometry>("/feedbackLoad",1);
            pubLoadDesieredPos = nh.advertise<nav_msgs::Odometry>("/desiredLoadPos",1);


//            subEncoder = nh.subscribe("/vicon/encoder/encoder",1,&Node::callbackEncoder,this);

#ifdef PX4
            subIMU = nh.subscribe("/mavros/imu/data_raw", 1, &Node::callbackImu, this);
            subOdometry = nh.subscribe("/mavros/local_position/odom", 1, &Node::callbackOdometry, this);
            subPX4State = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &Node::state_cb,this);
            subPayload = nh.subscribe("/uav/load/estimation/encoder", 1, &Node::callbackPayload, this);

            pubDrop = nh.advertise<std_msgs::UInt8>("/is_drop",10);

            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
#endif

#ifdef VREP
            subIMU = nh.subscribe("/imu", 1, &Node::callbackImu, this);
            subOdometry = nh.subscribe("/odom", 1, &Node::callbackOdometry, this);
            subPayload = nh.subscribe("/payload", 1, &Node::callbackPayload, this);

            //    使用视觉估计的位置
//            subPixelError = nh.subscribe("/pixelerror", 1, &Node::callbackPixelError,this);
            pubRotorRevs = nh.advertise<std_msgs::Float64MultiArray>("/rotorRevs", 1);
#endif
        }

        void publishToVrep(){
            Vec4 revs = controller.getRevs();
            std_msgs::Float64MultiArray revsArray;
            for(int i = 0 ; i < 4 ; i ++ ){
                if(isnanl(revs[i])){
                    revs[i] = 0;
                }
            }

            if(controller.isDrop)
            {
                revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w(),1};
            }
            else
            {
                revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w(),0};
            }


            pubRotorRevs.publish(revsArray);
            pubDesiredPos.publish(controller.desieredPos);
            pubFeedbackLoad.publish(controller.feedbackLoad);
            pubLoadDesieredPos.publish(controller.desieredLoadPos);
        }

        void publishToPX4(){
            mavros_msgs::AttitudeTarget AttTar;
            AttTar.header.stamp = ros::Time::now();
            AttTar.thrust = controller.getLiftForce()/41;
            AttTar.orientation = controller.getQuaternion();
            pubAttComm.publish(AttTar);
            pubDesiredPos.publish(controller.desieredPos);
            pubFeedbackLoad.publish(controller.feedbackLoad);
            pubLoadDesieredPos.publish(controller.desieredLoadPos);
        }
    };
}

#endif //LOADCONTROLLER_NODE_H
