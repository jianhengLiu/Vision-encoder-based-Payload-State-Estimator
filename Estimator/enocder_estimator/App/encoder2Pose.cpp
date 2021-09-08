//
// Created by nrsl on 8/8/20.
//

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include "fstream"
#include "std_msgs/UInt8.h"
#include "eigen3/Eigen/Dense"
const double PI = 3.14159265;
using namespace std;
//创建一个serial类
serial::Serial sp;
queue<geometry_msgs::PoseStamped> msgQueue;

queue<std_msgs::Float32MultiArray> angQueue;
ros::Publisher encoderPub;
ros::Publisher anglePub;
double rad2deg(double rad)
{
    return rad * 360 / 2 / PI;
}

void pubDataDelay(geometry_msgs::PoseStamped nowData,std_msgs::Float32MultiArray nowAng,double delayTime){

    nowData.header.stamp = ros::Time::now();
    nowData.header.frame_id = "map";
    msgQueue.push(nowData);
    angQueue.push(nowAng);
    if(!msgQueue.empty()){
//        printf("%d %lf\n",msgQueue.size(), nowData.header.stamp.toSec() -msgQueue.front().header.stamp.toSec() );

        while(nowData.header.stamp.toNSec() -msgQueue.front().header.stamp.toNSec()  > delayTime*1e6  )// 延迟100ms
        {
//            msgQueue.front().header.stamp = ros::Time::now();
            encoderPub.publish(msgQueue.front());
            anglePub.publish(angQueue.front());
            msgQueue.pop();
            angQueue.pop();
        }

    }

}
const uint8_t dropMsg[] = {'d','r','o','p'};
const uint8_t lockMsg[] = {'l','o','c','k'};
void dropMsgCallback(const std_msgs::UInt8::ConstPtr &msg){
    if(msg->data == 1){

        int writeBytes  = sp.write(dropMsg,sizeof(dropMsg));
        //cout<<"Droped!"<<writeBytes<<endl;
    }

    if(msg->data == 2){

        int writeBytes  = sp.write(lockMsg,sizeof(lockMsg));
        //cout<<"Locked!"<<writeBytes<<endl;
    }
}


double theta1s[10000];
double theta2s[10000];
int indexx = 0;

int main(int argc, char** argv)//argc是命令行总的参数个数
//argv[]为保存命令行参数的字符串指针，其中第0个参数是程序的全名，以后的参数为命令行后面跟的用户输入的参数
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;


    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);


    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)//捕捉输入输出异常
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    anglePub= n.advertise<std_msgs::Float32MultiArray>("/uav/load/estimation/encoder/angle", 1);
    encoderPub = n.advertise<geometry_msgs::PoseStamped>("/uav/load/estimation/encoder_real", 1);
    ros::Subscriber drop_sub;
    drop_sub = n.subscribe("/is_drop",1,dropMsgCallback);
    ros::Rate loop_rate(1000);
    float decode[2];
    float theta1,theta2;
//    float l = 0.72;//1.18;//sqrt(0.859250359444*0.859250359444+0.0274572322272*0.0274572322272+0.105366327853*0.105366327853);


    geometry_msgs::PoseStamped pubPose;
    std_msgs::Float32MultiArray pubAngle;
    pubAngle.data.resize(3);
    ofstream ofs("/home/kevin/UAV/ukf-code/data/data_encoder.txt");
    long cnt = 0;
    // double mu1 = 0,mu2 = 0;
    double body1_W[3] = {-994.776, 173.633, 191.667};
    double est[3] = {-971.712,350.304, 160.499};
    double body2_W[3] = {-977.416, 288.044, 193.320};
    double payload_W[3] = {-996.073, 1113.814, 43.039};
    double l = 0;
    for(int i = 0 ; i < 3 ; i ++){
        l+= ((payload_W[i] - body2_W[i])-(est[i] - body1_W[i]) )* ( (payload_W[i] - body2_W[i])-(est[i] - body1_W[i]) );
    }
    l = sqrt(l) / 1000;
    l = 0.57;//0.8;//0.72;
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n>=12)
        {
            uint8_t msg[12];
            //读出数据
            n = sp.read(msg, 12);
            for(int i = 0 ; i < 6 ; i ++ )
            {
                if(msg[i] == 'a' && msg[i+5] == 'g'){;
                    decode[0] = (uint16_t)(msg[i+2]<<8|msg[i+1]);
                    decode[1] = (uint16_t)(msg[i+4]<<8|msg[i+3]);
                    theta1 =-( decode[0]  / 16384.0f +(176.77)/360)* 2 * PI;//177.495)/360)* 2 * PI;
                    theta2 =-( decode[1]  / 16384.0f +(-63.479)/360)* 2 * PI ;//-60.3589)/360)* 2 * PI ;
                    while(theta1 > PI) theta1 -= 2*PI;
                    while(theta2 > PI) theta2 -= 2*PI;
                    while(theta1 < -PI) theta1 += 2*PI;
                    while(theta2 < -PI) theta2 += 2*PI;
//                     cout<<rad2deg(theta1)<<" "<<rad2deg(theta2)<<endl;

                    pubAngle.data[0] = theta1;
                    pubAngle.data[1] = theta2;
                    pubAngle.data[2] = l;
//	测量传感器方差
/*
		    theta1s[indexx] = (theta1);
		    theta2s[indexx] = (theta2);
		    mu1 += theta1s[i];
		    mu2 += theta2s[i];
		    indexx ++;
*/
                    pubPose.pose.position.z = sqrt(l*l/(1+tan(theta1)*tan(theta1) + tan(theta2)* tan(theta2)));
                    pubPose.pose.position.y = -pubPose.pose.position.z * tan(theta1);
                    pubPose.pose.position.x = pubPose.pose.position.z * tan(theta2);
// 卡尔曼滤波测试数据生成
//                    if(cnt++ % 2){
//                        ofs<<"E "<<theta1<<" "<<theta2<<" "<<ros::Time::now().toNSec()<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<endl;
//                    }else
//                    {
//                        ofs<<"V "<<pubPose.pose.position.x<<" "<<pubPose.pose.position.y<<" "<<pubPose.pose.position.z<<" "<<ros::Time::now().toNSec()<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<endl;
//                    }
//                    if(cnt++ % 2){
//                        ofs<<"L "<<theta1<<" "<<theta2<<" "<<ros::Time::now().toNSec()<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<endl;
//                    }else
//                    {
//                        ofs<<"R "<<theta1<<" "<<theta2<<" "<<ros::Time::now().toNSec()<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<endl;
//                    }
                    break;
                }
            }
        }
//	if(indexx>9999) break;
        pubDataDelay(pubPose,pubAngle,0);//70
        ros::spinOnce();
        loop_rate.sleep();
    }

// 测量传感器方差
/*
mu1 /= indexx;
mu2 /= indexx;
double s21=0,s22=0;
for(int i = 0 ; i < indexx - 1; i++){
	s21+=(theta1s[i] - mu1) * (theta1s[i] - mu1);
	s22+=(theta2s[i] - mu2) * (theta2s[i] - mu2);
}
cout<<mu1<<endl;
cout<<mu2<<endl;
cout<<s21/indexx<<endl;
cout<<s22/indexx<<endl;
*/
    ofs.close();
    //关闭串口
    sp.close();

    return 0;
}
