#include<iostream>
#include<ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Eigen>
#include <Eigen/Core>

#include<math.h>
#include<stdio.h>

using namespace std;
using namespace Eigen;
double t_acc ;
double t_ang ;

Quaterniond q(1.0, 0.0, 0.0, 0.0);
Vector3d vel;
Vector3d pos;
Vector3d acc;
Vector3d gra;

void line_acc_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
        sensor_msgs::Imu current_imu = *msg;
        double t_now = current_imu.header.stamp.sec + current_imu.header.stamp.nsec/1000000000.0;
        double t =  (t_now - t_acc);
        t_acc = t_now;
        if(t>0.1) return;

        double ax = current_imu.linear_acceleration.x*1.033;
        double ay = current_imu.linear_acceleration.y*1.033;
        double az = current_imu.linear_acceleration.z*1.033;
        acc<<ax,ay,az;
        cout<<"line: "<<t<<" "<<ax<<" "<<ay<<" "<<az<<endl;  

        //速度解算
        vel = vel + (q.toRotationMatrix()*acc+gra)*t;
        //位置解算
        pos = pos + vel*t;
        cout<<"acc1:"<<acc.norm()<<" "<<acc<<endl;
        cout<<"acc2: "<<q.toRotationMatrix()*acc<<endl;
        cout<<"acc: "<<q.toRotationMatrix()*acc+gra<<endl;
        cout<<"vel: "<<vel<<endl;
        cout<<"pos: "<<pos<<endl;
}

void ang_vel_cb(const sensor_msgs::Imu::ConstPtr& msg)
{

        sensor_msgs::Imu current_imu = *msg;
        double t_now = current_imu.header.stamp.sec + current_imu.header.stamp.nsec/1000000000.0;
        double t =  (t_now - t_ang);
        t_ang = t_now;
        if(t>0.1) return;
        double wx = current_imu.angular_velocity.x;
        double wy = current_imu.angular_velocity.y;
        double wz = current_imu.angular_velocity.z;

        Vector3d dw;
        dw <<wx*t,wy*t,wz*t;
        double fai = dw.norm();
        if(fai==0) return;
        double v1 = cos(fai);
        Vector3d v = dw / fai * sin(fai * 0.5);
        Quaterniond dq(v1, v(0), v(1), v(2));
        q = q*dq;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"imu_cal");
	ros::NodeHandle nh("~");

    vel<<0,0,0;
    pos<<0,0,0;
    gra<<0,0,-9.8;

    ros::Subscriber acc_sub = nh.subscribe("/camera/accel/sample",1000, line_acc_cb);
    ros::Subscriber ang_sub = nh.subscribe("/camera/gyro/sample",1000,ang_vel_cb);

    ros::spin();
    cout<<"hello"<<endl;
}