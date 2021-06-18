//#include "FlightController/common_include.h"
#include "loadController/common_include.h"
#include "loadController/flightController.h"
#include "loadController/node.h"
#include "trajGenerator/trajectory_generator_waypoint.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>

#define PI 3.1415926535

ros::Subscriber pose_sub;
geometry_msgs::Pose InitPose;
typedef enum{
    takeoff,
    attctr,
    land
}workstate_t;
workstate_t workState;
int callBackCnt;
void poseCallback(const geometry_msgs::PoseStamped &msg){
    InitPose = msg.pose;
}

/**
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    uint8 rssi
    uint16[] channels
 * @param msg
 */

void rcCallback(const mavros_msgs::RCInConstPtr &msg){

    if(msg->channels[7] > 1800){    // D开关向上 降落
        workState = land;
    }else if(msg->channels[5]< 1500){ // 开关C向下
        workState=takeoff;
    }else if ( msg -> channels[5] > 1500){ // 开关C向上 使用姿态控制器
        workState = attctr;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);


    ros::Subscriber moCap = nh.subscribe
            ("/mavros/vision_pose/pose", 10, poseCallback);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Subscriber rcSub = nh.subscribe
            ("/mavros/rc/in", 10, rcCallback);
    //the setpoint publishing rate MUST be faster than 2Hz

//    int cnt = 10;
//    ros::Rate initRate(1);
//    while(cnt--){
//        ros::spinOnce();
//        initRate.sleep();
//    }
    geometry_msgs::PoseStamped pose,pose2;
    geometry_msgs::TwistStamped landSignal;
    memset(&landSignal, 0 ,sizeof(landSignal));
    landSignal.twist.linear.z = -0.3;
    landSignal.twist.linear.x = 0;
    landSignal.twist.linear.y = 0;
    pose.pose = InitPose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.3;
    pose2.pose = InitPose;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 1.5;
    pose2.pose.position.z = 1.3;
    ros::Rate rate(1000);
    pose.header.frame_id = "body";

    wtr::Node n;


    std::vector<TrajectoryGeneratorWaypoint> vect_Traj;

    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.2;
    _traj_vis.scale.y = 0.2;
    _traj_vis.scale.z = 0.2;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    _traj_vis.points.clear();

    visualization_msgs::Marker line_list;
    int id = 0;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "wp_path";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.pose.orientation.x = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;

    line_list.id = id;

    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;

    line_list.points.clear();


    {
        std::string config_file = "/home/chrisliu/NewDisk/1011_boom_src/src/Controller/main_controller/config/default.yaml";

        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        cv::Mat readMat;
        fs["waypointMatrix"] >> readMat;
        Eigen::MatrixXd readMat_e(9, 4);
        cv::cv2eigen(readMat, readMat_e);
//        cout<<readMat<<endl;
        for (int i = 0; i < readMat_e.rows() - 1; i++) {
            TrajectoryGeneratorWaypoint TrajectoryGenerator(2,1.5);

            nav_msgs::Path waypoints;
            waypoints.header.frame_id = std::string("world");
            waypoints.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped temp_wpt;
            temp_wpt.header.frame_id = "map";
            temp_wpt.pose.orientation.w = 1;
            temp_wpt.pose.orientation.z = 0;
            temp_wpt.pose.orientation.y = 0;
            temp_wpt.pose.orientation.x = 0;


            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = readMat_e(i, 0);
            temp_wpt.pose.position.y = readMat_e(i, 1);
            temp_wpt.pose.position.z = readMat_e(i, 2);
            waypoints.poses.push_back(temp_wpt);

            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = readMat_e(i + 1, 0);
            temp_wpt.pose.position.y = readMat_e(i + 1, 1);
            temp_wpt.pose.position.z = readMat_e(i + 1, 2);
            waypoints.poses.push_back(temp_wpt);

            Eigen::MatrixXd margin_constraint(2,3);
            margin_constraint = readMat_e.block(i,3,2,6);

            TrajectoryGenerator.isTraj = TrajectoryGenerator.trajGeneration(waypoints,margin_constraint);
            std::cout << "_waypoints" << std::endl;
            std::cout << TrajectoryGenerator._waypoints << std::endl;
            if (TrajectoryGenerator.isTraj) {
//                for(int i =0;i<TrajectoryGenerator.visWayPointPath().points.size();i++)
//                {
//                    line_list.points.push_back(TrajectoryGenerator.visWayPointPath().points.at(i));
//                }
//
//                for(int i =0;i<TrajectoryGenerator.visWayPointTraj().points.size();i++)
//                {
//                    _traj_vis.points.push_back(TrajectoryGenerator.visWayPointTraj().points.at(i));
//                }
                vect_Traj.push_back(TrajectoryGenerator);
            }
        }
//        if(!vect_Traj.empty())
//        {
//            vect_Traj[0]._wp_path_vis_pub.publish(line_list);
//            vect_Traj[0]._wp_traj_vis_pub.publish(_traj_vis);
//        }
    }



    double massQuadcopter = 0.52;//1.65;//小电池  //中电池2
    double massPayload = 0.168;
    float length = 0.72;//0.01;
    Mat33 II;
    II<< 0.0347, 0, 0,
            0, 0.0458, 0,
            0, 0,0.0977;
    n.controller.initializeParameter(massQuadcopter, massPayload, length, II);

    workState = attctr;
    double startTime = 0;
    int cnt_traj = 0;
    std_msgs::UInt8 isDrop;
    isDrop.data = 2;

//    n.controller.isDrop = true;
    while(ros::ok()){
        workState = attctr;
        switch (workState) {
            case takeoff:
            {
                n.controller.accumulateError = Vec3(0,0,0);
                pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(pose);
                isDrop.data = 2;
                break;
            }
            case land:
            {
                landSignal.header.stamp = ros::Time::now();
                landSignal.header.frame_id="body";
                speed_pub.publish(landSignal);
                break;
            }
            case attctr:
            {
                /*n.controller.accumulateError = Vec3(0,0,0);
                pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(pose2);
                isDrop.data = 2;
                break;*/


//                启动姿态控制器
                if(vect_Traj.empty())
                {
//                    Vec3 desierPosition = Vec3(0,0,0.58);
//                    Vec3 desierPayloadAcc = Vec3(0,0,0);
//                    n.controller.LiuController(desierPosition,desierPayloadAcc);

                    Vec3 inputDesiredPos = Vec3(0,0,1.58);
                    Vec3 inputDesiredVel = Vec3(0,0,0);
                    Vec3 inputDesiredAcc = Vec3(0,0,0);
                    Eigen::Vector3d inputDesiredJerk = Vec3(0,0,0);
                    Eigen::Vector3d inputDesiredSnap = Vec3(0,0,0);
                    n.controller.LiuController(inputDesiredPos,inputDesiredVel,inputDesiredAcc,inputDesiredJerk,inputDesiredSnap);

                } else
                {
                    if(startTime == 0)
                    {
                        startTime = ros::Time::now().toSec();
                    }
                    double t = ros::Time::now().toSec()-startTime;
//                    if(1)
                    if(vect_Traj[cnt_traj]._totalTime>t)
                    {
//                        Vec3 desierPayloadPos = vect_Traj[cnt_traj].getTrajectoryStates(t,0);
//                        Vec3 desierPayloadAcc = vect_Traj[cnt_traj].getTrajectoryStates(t,2);
//                        n.controller.LiuController(desierPayloadPos,desierPayloadAcc);
//                        double delta = 2;
//                        Vec3 inputDesiredPos = Vec3(t/delta,sin(t)/delta, 1.58);
//                        Vec3 inputDesiredVel = Vec3(1/delta, cos(t)/delta, 0);
//                        Vec3 inputDesiredAcc = Vec3(0, -sin(t)/delta, 0);
//                        Eigen::Vector3d inputDesiredJerk = Vec3(0, -cos(t)/delta, 0);
//                        Eigen::Vector3d inputDesiredSnap = Vec3(0, sin(t)/delta, 0);


                        Eigen::Vector3d inputDesiredPos = vect_Traj[cnt_traj].getTrajectoryStates(t,0);
                        Eigen::Vector3d inputDesiredVel = vect_Traj[cnt_traj].getTrajectoryStates(t,1);
                        Eigen::Vector3d inputDesiredAcc = vect_Traj[cnt_traj].getTrajectoryStates(t,2);
                        Eigen::Vector3d inputDesiredJerk = vect_Traj[cnt_traj].getTrajectoryStates(t,3);
                        Eigen::Vector3d inputDesiredSnap = vect_Traj[cnt_traj].getTrajectoryStates(t,4);
                        n.controller.LiuController(inputDesiredPos,inputDesiredVel,inputDesiredAcc,inputDesiredJerk,inputDesiredSnap);

                        if(abs(n.controller.feedback.cPayloadVelocity.x())<0.05&&n.controller.feedback.cPayloadPosition.x()>3.5)
                        {
#ifdef PX4
                            isDrop.data = 1;
#endif
                            n.controller.accumulateError = Vec3(0,0,0);
                            n.controller.isDrop = true;
                        }
                    }else
                    {
                        t = vect_Traj[cnt_traj]._totalTime-0.001;
//                        Vec3 desierPayloadPos = vect_Traj[cnt_traj].getTrajectoryStates(t,0);
//                        Vec3 desierPayloadAcc = Vec3(0, 0, 0);
//                        n.controller.LiuController(desierPayloadPos,desierPayloadAcc);

                        Eigen::Vector3d inputDesiredPos = vect_Traj[cnt_traj].getTrajectoryStates(t,0);
                        Eigen::Vector3d inputDesiredVel = vect_Traj[cnt_traj].getTrajectoryStates(t,1);
                        Eigen::Vector3d inputDesiredAcc = vect_Traj[cnt_traj].getTrajectoryStates(t,2);
                        Eigen::Vector3d inputDesiredJerk = vect_Traj[cnt_traj].getTrajectoryStates(t,3);
                        Eigen::Vector3d inputDesiredSnap = vect_Traj[cnt_traj].getTrajectoryStates(t,4);
                        n.controller.LiuController(inputDesiredPos,inputDesiredVel,inputDesiredAcc,inputDesiredJerk,inputDesiredSnap);

                        if(cnt_traj+1<vect_Traj.size())
                        {
                            startTime += vect_Traj[cnt_traj]._totalTime;
                            cnt_traj++;
                        }

                    }

                }
#ifdef PX4
                n.publishToPX4();
n.pubDrop.publish(isDrop);
#endif
#ifdef VREP
                n.publishToVrep();
#endif

                break;
            }
            default:
            {
                pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(pose);
                break;
            }

        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

