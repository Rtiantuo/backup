#ifndef DWA_H
#define DWA_H

#include<iostream>
#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/StdVector>
#include<tf/tf.h>
#include<cmath>
using namespace std;

ros::Publisher path_pub;
ros::Publisher vel_pub;
ros::Subscriber vel_sub;
double vx=0;
double wz=0;
double cal_vx;
double  cal_wz;
double free_vx;
double free_wz;

double toX;
double toY;
double toHeading;

struct Point
{
    double x,y;
    Point(double x,double y)
    {
        this->x=x;
        this->y=y;
    }
};
//机器人状态参数
struct RobotState
{
    Eigen::Vector3d position;
    double yaw;
    Eigen::Vector2d goal_now;
};

//机器人动力学参数
struct RobotParam
{
    double v_max;
    double w_max;
    double v_acc_max;
    double w_acc_max;
};

//DWA相关参数
struct DwaParam
{
    Eigen::Vector3d evalCB;  //heading vel dist
    Eigen::Vector2d smaple_num;  //[vx,w]
    double dt;
    double plan_t;
};

RobotState robot_state;
RobotParam robot_param;
DwaParam dwa_param;

void init()
{
    robot_param.v_max=0.2;
    robot_param.v_acc_max=0.2;
    robot_param.w_max=3.14;
    robot_param.w_acc_max=6.28;
    dwa_param.evalCB<<0.1,5,5;   //heading  vel  dist
    dwa_param.smaple_num<<10,20;
    dwa_param.dt=0.1;
    dwa_param.plan_t=1.5;
}

int near_pt(double x,double y);
RobotState cal_state(RobotState& state);
double toDegree(double theta);

#endif