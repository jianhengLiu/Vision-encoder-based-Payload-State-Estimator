#include "../include/trajGenerator/trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>



using namespace std;
using namespace Eigen;


void TrajectoryGeneratorWaypoint::waypoints_generator() {
	
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
    temp_wpt.pose.position.x = 0;
    temp_wpt.pose.position.y = 0;
    temp_wpt.pose.position.z = 1.3-0.92;
    waypoints.poses.push_back(temp_wpt);
//    goal_pub.publish(temp_wpt);
//    sleep(1);

    double dataset_x[]={1.05,1.5,5};
    double dataset_y[]={1.03,-0.76,0};
    for(int i = 0 ; i < sizeof(dataset_x)/sizeof(double); i++){
        temp_wpt.header.stamp = ros::Time::now();
        temp_wpt.pose.position.x = dataset_x[i];
        temp_wpt.pose.position.y = dataset_y[i];
        temp_wpt.pose.position.z = 1.3-0.92;
        waypoints.poses.push_back(temp_wpt);
////        goal_pub.publish(temp_wpt);
////        sleep(1);
    }

    /**
    * 记得最后给个负的
    * */
//    temp_wpt.header.stamp = ros::Time::now();
//    temp_wpt.pose.position.x = 0;
//    temp_wpt.pose.position.y = 0;
//    temp_wpt.pose.position.z = -4;
//    waypoints.poses.push_back(temp_wpt);
//    goal_pub.publish(temp_wpt);
//    sleep(1);
    {
        isTraj = trajGeneration(waypoints);
        cout << "_waypoints" << endl;
        cout << _waypoints << endl;
        if (isTraj) {
            //获取可视化数据
            _wp_path_vis_pub.publish(visWayPointPath());
            sleep(1);
            _wp_traj_vis_pub.publish(visWayPointTraj());
//        sleep(1);
        }
    }
}


void TrajectoryGeneratorWaypoint::input_waypoint(nav_msgs::Path &waypoints,geometry_msgs::PoseStamped msg) {

    if (msg.pose.position.z >= 0) {
        // if height >= 0, it's a normal goal;
        geometry_msgs::PoseStamped pt = msg;

        waypoints.poses.push_back(pt);
    } else if (msg.pose.position.z > -1.0) {
        // if 0 > height > -1.0, remove last goal;
        if (waypoints.poses.size() >= 1) {
            waypoints.poses.erase(std::prev(waypoints.poses.end()));
        }
    }
}

bool TrajectoryGeneratorWaypoint::trajGeneration(nav_msgs::Path wp,Eigen::MatrixXd margin_constraint)
{
    bool isGenerated = false;
    std::vector<Eigen::Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int) wp.poses.size(); k++)
    {
        Eigen::Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if (wp.poses[k].pose.position.z < 0.0)
            break;
    }

    /**
     * 原添加了（0,0,1.1)的初始点
     */
//    Eigen::MatrixXd waypoints(wp_list.size() + 1, 3);
//    //导入初始位置
//    Eigen::Vector3d _startPos = Eigen::Vector3d(0,0,1.1);
//    waypoints.row(0) = _startPos;
//
//    for (int k = 0; k < (int) wp_list.size(); k++)
//        waypoints.row(k + 1) = wp_list[k];

    /**
     * 这个没有
     */
    Eigen::MatrixXd waypoints(wp_list.size(), 3);
    for (int k = 0; k < (int) wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];

    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    //启动轨迹生成

    _waypoints = waypoints;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    timeAllocation(_waypoints);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    PolyQPGeneration(_waypoints,margin_constraint);

    isGenerated = true;
    return isGenerated;

}


TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(double Vel, double Acc)
{
    _Vel = Vel;
    _Acc = Acc;
    /**
     * ROS
    */
    //订阅路径节点
    _way_pts_sub = nh.subscribe("/waypoint_generator/waypoints", 1, &TrajectoryGeneratorWaypoint::rcvWaypointsCallBack,this);

    //发布可视化节点
    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal",1);
}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint()
{}

void TrajectoryGeneratorWaypoint::init(double Vel, double Acc) {
    _Vel = Vel;
    _Acc = Acc;
}

//define factorial function, input i, output i!
double TrajectoryGeneratorWaypoint::Factorial(int x)
{
    double fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

//void TrajectoryGeneratorWaypoint::PolyQPGeneration(
//        const Eigen::MatrixXd &Path)            // boundary jerk
//{
//    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
//    int p_order = 2 * d_order - 1;              // the order of polynomial
//    int p_num1d = p_order + 1;                  // the number of variables in each segment
//
//    int m = _polyTime.size();                          // the number of segments
//
//    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
//
////    getQ
////先只计算一个维度的Q，因为我们取三个维度的时间间隔是一样的
////即Q_x = Q_y = Q_z
//    MatrixXd Q = MatrixXd::Zero(m * p_num1d, m * p_num1d);
//    MatrixXd Qj;
//    for (int j = 0; j <= m - 1; ++j)
//    {
//        Qj = MatrixXd::Zero(p_num1d, p_num1d);
//        for (int i = d_order; i <= p_order; ++i)
//        {
//            for (int l = i; l <= p_order; ++l)
//            {
//                Qj(i, l) =
//                        Factorial(i) / Factorial(i - d_order) * Factorial(l) / Factorial(l - d_order) *
//                        pow(_polyTime(j), i + l - p_order) / (i + l - p_order);
//
//                Qj(l, i) = Qj(i, l);
//
//            }
//        }
//        Q.block(j * p_num1d, j * p_num1d, p_num1d, p_num1d) = Qj;
//    }
//
//    /*   Produce the dereivatives in X, Y and Z axis directly.  */
//    //  getM
//    //先只计算一个维度的M，因为我们取三个维度的时间间隔是一样的，所以可以通过拼接减少运算量
//    //即M_x = M_y = M_z
//    Eigen::MatrixXd M = MatrixXd::Zero(m * p_num1d, m * p_num1d);
//    for (int j = 0; j <= m - 1; ++j)
//    {
//        for (int k = 0; k <= d_order - 1; ++k)
//        {
//            M(k + j * p_num1d, k + j * p_num1d) = Factorial(k);
//            for (int i = k; i <= p_order; ++i)
//            {
//                M(d_order + k + j * p_num1d, i + j * p_num1d) =
//                        Factorial(i) / Factorial(i - k) * pow(_polyTime(j), i - k);
//            }
//        }
//    }
//
////    getCt
////  Ct  for start point
//    Eigen::MatrixXd Ct_start = MatrixXd::Zero(d_order, d_order * (m + 1));
////block 矩阵块（起始位置x，y,大小行，列）
//    Ct_start.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);
//
//    //  Ct  for middle point
//    Eigen::MatrixXd Ct_mid = MatrixXd::Zero(2 * d_order * (m - 1), d_order * (m + 1));
//    Eigen::MatrixXd Cj;
//    double start_idx_2 = 0;
//    double start_idx_1 = 0;
//    for (int j = 0; j <= m - 2; ++j)
//    {
//        Cj = MatrixXd::Zero(d_order, d_order * (m + 1));
//        Cj(0, d_order + j) = 1;
//        start_idx_2 = 2 * d_order + m - 1 + (d_order-1) * j;
//        Cj.block(1, start_idx_2, d_order - 1, d_order - 1) = MatrixXd::Identity(d_order - 1, d_order - 1);
//        start_idx_1 = 2 * d_order * j;
//        Eigen::MatrixXd Cj_adjacen(2 * d_order, d_order * (m + 1));
//        Cj_adjacen << Cj,
//                Cj;
//        Ct_mid.block(start_idx_1, 0, 2 * d_order, d_order * (m + 1)) = Cj_adjacen;
//    }
//
//    //  Ct  for end point
//    Eigen::MatrixXd Ct_end = MatrixXd::Zero(d_order, d_order * (m + 1));
//    Ct_end.block(0, d_order + m - 1, d_order, d_order) = MatrixXd::Identity(d_order, d_order);
//
//    Eigen::MatrixXd Ct(2 * d_order * m, d_order * (m + 1));
//    Ct << Ct_start,
//            Ct_mid,
//            Ct_end;
//
//
//
//    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
////    即对应的R矩阵
//
//    Eigen::MatrixXd C = Ct.transpose();
//    Eigen::MatrixXd R = C * M.inverse().transpose() * Q * M.inverse() * Ct;
//
////   中间点m-1个，始末点6个边界约束p0,v1,a2,j3,snap4,x5
////   (m-1)+2x6 = m+11个约束条件
////  待确定的量每个中间点三个,v1,a2,j3,snap4,x5
////  5*(m-1)
////  所以总长度
//    int n_boundary_constraint = (m-1)+2*d_order;
//    int n_continuity_constraint = (d_order-1)*(m-1);
//
//    Eigen::MatrixXd R_pp = R.block(n_boundary_constraint, n_boundary_constraint, n_continuity_constraint, n_continuity_constraint);
//    Eigen::MatrixXd R_fp = R.block(0, n_boundary_constraint, n_boundary_constraint, n_continuity_constraint);
//
//
//    VectorXd dF_x = get_dF(0, m, Path);
//    VectorXd dF_y = get_dF(1, m, Path);
//    VectorXd dF_z = get_dF(2, m, Path);
//
//    VectorXd dP_x = -R_pp.inverse() * R_fp.transpose() * dF_x;
//    VectorXd dP_y = -R_pp.inverse() * R_fp.transpose() * dF_y;
//    VectorXd dP_z = -R_pp.inverse() * R_fp.transpose() * dF_z;
//
//    VectorXd d_x(dF_x.size() + dP_x.size(), 1);
//    d_x << dF_x,
//            dP_x;
//
//    VectorXd d_y(dF_y.size() + dP_y.size());
//    d_y << dF_y,
//            dP_y;
//
//    VectorXd d_z(dF_z.size() + dP_z.size());
//    d_z << dF_z,
//            dP_z;
//
//    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);
//
//    VectorXd P_x = M.inverse() * Ct * d_x;
//    VectorXd P_y = M.inverse() * Ct * d_y;
//    VectorXd P_z = M.inverse() * Ct * d_z;
//
//    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);// position(x,y,z), so we need (3 * p_num1d) coefficients
//    for (int i = 0; i < m; ++i)
//    {
//        PolyCoeff.block(i, 0, 1, p_num1d) = P_x.block(i * p_num1d, 0, p_num1d, 1).transpose();
//    }
//    for (int i = 0; i < m; ++i)
//    {
//        PolyCoeff.block(i, p_num1d, 1, p_num1d) = P_y.block(i * p_num1d, 0, p_num1d, 1).transpose();
//    }
//    for (int i = 0; i < m; ++i)
//    {
//        PolyCoeff.block(i, 2 * p_num1d, 1, p_num1d) = P_z.block(i * p_num1d, 0, p_num1d, 1).transpose();
//    }
//
//    _polyCoeff = PolyCoeff;
//}

void TrajectoryGeneratorWaypoint::PolyQPGeneration(
        const Eigen::MatrixXd &Path,Eigen::MatrixXd margin_constraint)            // boundary jerk
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order = 2 * d_order - 1;              // the order of polynomial
    int p_num1d = p_order + 1;                  // the number of variables in each segment

    int m = _polyTime.size();                          // the number of segments

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */

//    getQ
//先只计算一个维度的Q，因为我们取三个维度的时间间隔是一样的
//即Q_x = Q_y = Q_z
    MatrixXd Q = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    MatrixXd Qj;
    for (int j = 0; j <= m - 1; ++j)
    {
        Qj = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = d_order; i <= p_order; ++i)
        {
            for (int l = i; l <= p_order; ++l)
            {
                Qj(i, l) =
                        Factorial(i) / Factorial(i - d_order) * Factorial(l) / Factorial(l - d_order) *
                        pow(_polyTime(j), i + l - p_order) / (i + l - p_order);

                Qj(l, i) = Qj(i, l);

            }
        }
        Q.block(j * p_num1d, j * p_num1d, p_num1d, p_num1d) = Qj;
    }

    /*   Produce the dereivatives in X, Y and Z axis directly.  */
    //  getM
    //先只计算一个维度的M，因为我们取三个维度的时间间隔是一样的，所以可以通过拼接减少运算量
    //即M_x = M_y = M_z
    Eigen::MatrixXd M = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    for (int j = 0; j <= m - 1; ++j)
    {
        for (int k = 0; k <= d_order - 1; ++k)
        {
            M(k + j * p_num1d, k + j * p_num1d) = Factorial(k);
            for (int i = k; i <= p_order; ++i)
            {
                M(d_order + k + j * p_num1d, i + j * p_num1d) =
                        Factorial(i) / Factorial(i - k) * pow(_polyTime(j), i - k);
            }
        }
    }

//    getCt
//  Ct  for start point
    Eigen::MatrixXd Ct_start = MatrixXd::Zero(d_order, d_order * (m + 1));
//block 矩阵块（起始位置x，y,大小行，列）
    Ct_start.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    //  Ct  for middle point
    Eigen::MatrixXd Ct_mid = MatrixXd::Zero(2 * d_order * (m - 1), d_order * (m + 1));
    Eigen::MatrixXd Cj;
    double start_idx_2 = 0;
    double start_idx_1 = 0;
    for (int j = 0; j <= m - 2; ++j)
    {
        Cj = MatrixXd::Zero(d_order, d_order * (m + 1));
        Cj(0, d_order + j) = 1;
        start_idx_2 = 2 * d_order + m - 1 + (d_order-1) * j;
        Cj.block(1, start_idx_2, d_order - 1, d_order - 1) = MatrixXd::Identity(d_order - 1, d_order - 1);
        start_idx_1 = 2 * d_order * j;
        Eigen::MatrixXd Cj_adjacen(2 * d_order, d_order * (m + 1));
        Cj_adjacen << Cj,
                Cj;
        Ct_mid.block(start_idx_1, 0, 2 * d_order, d_order * (m + 1)) = Cj_adjacen;
    }

    //  Ct  for end point
    Eigen::MatrixXd Ct_end = MatrixXd::Zero(d_order, d_order * (m + 1));
    Ct_end.block(0, d_order + m - 1, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    Eigen::MatrixXd Ct(2 * d_order * m, d_order * (m + 1));
    Ct << Ct_start,
            Ct_mid,
            Ct_end;



    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
//    即对应的R矩阵

    Eigen::MatrixXd C = Ct.transpose();
    Eigen::MatrixXd R = C * M.inverse().transpose() * Q * M.inverse() * Ct;

//   中间点m-1个，始末点6个边界约束p0,v1,a2,j3,snap4,x5
//   (m-1)+2x6 = m+11个约束条件
//  待确定的量每个中间点三个,v1,a2,j3,snap4,x5
//  5*(m-1)
//  所以总长度
    int n_boundary_constraint = (m-1)+2*d_order;
    int n_continuity_constraint = (d_order-1)*(m-1);

    Eigen::MatrixXd R_pp = R.block(n_boundary_constraint, n_boundary_constraint, n_continuity_constraint, n_continuity_constraint);
    Eigen::MatrixXd R_fp = R.block(0, n_boundary_constraint, n_boundary_constraint, n_continuity_constraint);




    VectorXd dF_x = get_dF(0, m, Path,margin_constraint);
    VectorXd dF_y = get_dF(1, m, Path,margin_constraint);
    VectorXd dF_z = get_dF(2, m, Path,margin_constraint);

    VectorXd dP_x = -R_pp.inverse() * R_fp.transpose() * dF_x;
    VectorXd dP_y = -R_pp.inverse() * R_fp.transpose() * dF_y;
    VectorXd dP_z = -R_pp.inverse() * R_fp.transpose() * dF_z;

    VectorXd d_x(dF_x.size() + dP_x.size(), 1);
    d_x << dF_x,
            dP_x;

    VectorXd d_y(dF_y.size() + dP_y.size());
    d_y << dF_y,
            dP_y;

    VectorXd d_z(dF_z.size() + dP_z.size());
    d_z << dF_z,
            dP_z;

    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    VectorXd P_x = M.inverse() * Ct * d_x;
    VectorXd P_y = M.inverse() * Ct * d_y;
    VectorXd P_z = M.inverse() * Ct * d_z;

    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);// position(x,y,z), so we need (3 * p_num1d) coefficients
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, 0, 1, p_num1d) = P_x.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, p_num1d, 1, p_num1d) = P_y.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, 2 * p_num1d, 1, p_num1d) = P_z.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }

    _polyCoeff = PolyCoeff;
}

/*
 * 把所有除了位置的边界条件都设为0
 */
//VectorXd
//TrajectoryGeneratorWaypoint::get_dF(int k, int m,const Eigen::MatrixXd &Path)        // waypoints coordinates (3d))
//{
//    int n_boundary_constraint = (m-1)+2*d_order;
//
//    VectorXd dF = VectorXd::Zero(n_boundary_constraint);
//
//    for (int i = 0; i <= m; ++i)
//    {
//        if (i == 0)
//        {
//            dF(0) = Path(0, k);
//        } else if (i == m)
//        {
//            dF(n_boundary_constraint-d_order) = Path(i, k);
//        } else
//        {
//            dF(d_order + i - 1) = Path(i, k);
//        }
//
//    }
//    return dF;
//}

VectorXd
TrajectoryGeneratorWaypoint::get_dF(int k, int m,const Eigen::MatrixXd &Path,Eigen::MatrixXd margin_constraint)        // waypoints coordinates (3d))
{
    int n_boundary_constraint = (m-1)+2*d_order;

    VectorXd dF = VectorXd::Zero(n_boundary_constraint);

    for (int i = 0; i <= m; ++i)
    {
        if (i == 0)
        {
            dF(0) = Path(0, k);
            dF(1) = margin_constraint(0,k);
            dF(2) = margin_constraint(0,k+3);
        } else if (i == m)
        {
            dF(n_boundary_constraint-d_order) = Path(i, k);
            dF(n_boundary_constraint-d_order+1) = margin_constraint(1,k);
            dF(n_boundary_constraint-d_order+2) = margin_constraint(1,k+3);
        } else
        {
            dF(d_order + i - 1) = Path(i, k);
        }

    }
    return dF;
}

double TrajectoryGeneratorWaypoint::timeAllocation_1D(double dis)
{
    double T = 0;
    if (dis <= _Vel * _Vel / _Acc)
    {
        T = 2 * sqrt(dis / _Acc);
    } else
    {
        T = _Vel / _Acc + (dis) / _Vel;
    }
    return T;
}

void TrajectoryGeneratorWaypoint::timeAllocation(MatrixXd Path)
{
//    cout<<Path<<endl;
    VectorXd time(Path.rows() - 1);
    _totalTime = 0;

    /*

    STEP 1: Learn the "trapezoidal velocity" of "TIme Allocation" in L5, then finish this timeAllocation function

    variable declaration: _Vel, _Acc: _Vel = 1.0, _Acc = 1.0 in this homework, you can change these in the test.launch

    You need to return a variable "time" contains time allocation, which's type is VectorXd

    The time allocation is many relative timeline but not one common timeline

    */

//    trapezoidal velocity
    double delta_x = 0;
    double delta_y = 0;
    double delta_z = 0;

    for (int i = 0; i < time.size(); ++i)
    {
        delta_x = fabs(Path(i + 1, 0) - Path(i, 0));
        delta_y = fabs(Path(i + 1, 1) - Path(i, 1));
        delta_z = fabs(Path(i + 1, 2) - Path(i, 2));

        double dis = sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z);
        time(i) = timeAllocation_1D(dis);

        _totalTime += time(i);
    }
    cout << "time:" << endl << time << endl;
    _polyTime = time;
}


Vector3d TrajectoryGeneratorWaypoint::getPolyStates(int k, double t_seg, int order)
{

    int _poly_num1D = _polyCoeff.cols() / 3;

    Vector3d ret;

    for (int dim = 0; dim < 3; dim++)
    {
        VectorXd coeff = (_polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);

        for (int j = order; j < _poly_num1D; j++)
            if (j == 0)
                time(j) = Factorial(j) * 1.0;
            else
                time(j) = Factorial(j) / Factorial(j - order) * pow(t_seg, j - order);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getTrajectoryStates(double time_from_start, int order)
{
//    每一段轨迹的时间起始于0
//    t所在的时间段的初始时间
    double t_init = 0;
//    t在其对应时间段上的时间
    double t_seg = 0;

    //找出对应时间对应的第几段轨迹
    int seg_idx = 0;

    for (int i = 0; i < _polyTime.size(); i++)
    {
        if (time_from_start >= t_init + _polyTime(i))
        {
            t_init += _polyTime(i);
        } else
        {
            t_seg = time_from_start - t_init;
            seg_idx = i;
            break;
        }
    }

    Vector3d states = getPolyStates(seg_idx, t_seg, order);

    return states;
}

bool TrajectoryGeneratorWaypoint::trajGeneration(const nav_msgs::PathConstPtr wp,
                                                 Eigen::MatrixXd margin_constraint)
{
    bool isGenerated = false;
    std::vector<Eigen::Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int) wp->poses.size(); k++)
    {
        Eigen::Vector3d pt(wp->poses[k].pose.position.x, wp->poses[k].pose.position.y, wp->poses[k].pose.position.z);
        wp_list.push_back(pt);

        if (wp->poses[k].pose.position.z < 0.0)
            break;
    }

//    Eigen::MatrixXd waypoints(wp_list.size() + 1, 3);
//    导入初始位置
//    Eigen::Vector3d _startPos = Eigen::Vector3d(0,0,1.1);
//    waypoints.row(0) = _startPos;
//
//    for (int k = 0; k < (int) wp_list.size(); k++)
//        waypoints.row(k + 1) = wp_list[k];

    Eigen::MatrixXd waypoints(wp_list.size(), 3);
//    导入初始位置
//    Eigen::Vector3d _startPos = Eigen::Vector3d(0,0,1.1);
//    waypoints.row(0) = _startPos;

    for (int k = 0; k < (int) wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];

    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    //启动轨迹生成

    _waypoints = waypoints;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    timeAllocation(_waypoints);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    PolyQPGeneration(_waypoints);

    isGenerated = true;
    return isGenerated;

}

visualization_msgs::Marker TrajectoryGeneratorWaypoint::visWayPointPath()
{
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

    for (int i = 0; i < _waypoints.rows(); i++)
    {
        geometry_msgs::Point p;
        p.x = _waypoints(i, 0);
        p.y = _waypoints(i, 1);
        p.z = _waypoints(i, 2);

        if (i < (_waypoints.rows() - 1))
        {
            geometry_msgs::Point p_line;
            p_line = p;
            line_list.points.push_back(p_line);
            p_line.x = _waypoints(i + 1, 0);
            p_line.y = _waypoints(i + 1, 1);
            p_line.z = _waypoints(i + 1, 2);
            line_list.points.push_back(p_line);
        }
    }
    return line_list;
//    _wp_path_vis_pub.publish(line_list);
}

visualization_msgs::Marker TrajectoryGeneratorWaypoint::visWayPointTraj()
{
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

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    MatrixXd states(3, 3);
    Vector3d pos;
    geometry_msgs::Point pt;

    VectorXd time = _polyTime;

//    double t_temp = 0;
    for (int i = 0; i < time.size(); i++)
    {
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
//            states = getTrajectoryStates(polyCoeff, time, t+t_temp);
//            cout<<"states"<<endl<<states<<endl;
//            cur(0) = pt.x = states(0,0);
//            cur(1) = pt.y = states(0,1);
//            cur(2) = pt.z = states(0,2);

            pos = getPolyStates(i, t, 0);
            cur(0) = pt.x = pos(0);
            cur(1) = pt.y = pos(1);
            cur(2) = pt.z = pos(2);
//            cout<<pos<<endl;
            _traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
//        t_temp += time(i);
    }
    return _traj_vis;
//    publish(_traj_vis);
}
