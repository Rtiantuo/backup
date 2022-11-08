#include "string"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "mavros_msgs/PositionTarget.h"
#include "vector"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "random"
#include "Eigen/Eigen"
#include "Eigen/LU"
#include "../include/offb/PolyTraj.h"
#include "nav_msgs/Odometry.h"

mavros_msgs::State current_state;
nav_msgs::Odometry current_odom;
PolyTraj traj_exp_x;
PolyTraj traj_exp_y;
PolyTraj traj_exp_z;
PolyTraj traj_exp_yaw;
ros::Publisher pose_pub_;
double x_goal;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_odom = *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    /*状态更新*/
    if (current_state.mode != msg->mode && msg->mode == "OFFBOARD")//从任意mode进入OFFBOARD
    {
        /*设置当前odom为悬停位置*/
        traj_exp_x.set_point(current_odom.pose.pose.position.x,ros::Time::now().toSec());
        traj_exp_y.set_point(current_odom.pose.pose.position.y,ros::Time::now().toSec());
        traj_exp_z.set_point(current_odom.pose.pose.position.z,ros::Time::now().toSec());
        ROS_INFO("OFFBOARD MODE DETECTED HOVERING");
    }
    //从OFFBOARD切出到任意模式：无事发生
    current_state = *msg;
}

void set_pt_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    /*rviz指令响应函数*/
    if (current_state.mode != "OFFBOARD"){
        ROS_INFO("Not in OFFBOARD mode, cmd rejected");
        return;
    }
    else
        ROS_INFO("CMD RECEIVED");
    mavros_msgs::PositionTarget tar_msg;
    std::uniform_real_distribution<double> dis(-0.5,0.5);
    std::random_device rd;
    std::default_random_engine defaultRandomEngine{rd()};

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation,q);
    double rol_exp,pit_exp,yaw_exp;
    tf::Matrix3x3(q).getRPY(rol_exp, pit_exp, yaw_exp);

    tf::quaternionMsgToTF(current_odom.pose.pose.orientation,q);
    double rol_now,pit_now,yaw_now;
    tf::Matrix3x3(q).getRPY(rol_now,pit_now,yaw_now);

    Eigen::Vector3d p0, v0, a0, pf, vf, af;
    p0<< current_odom.pose.pose.position.x,current_odom.pose.pose.position.y,current_odom.pose.pose.position.z;
    v0<< current_odom.twist.twist.linear.x,current_odom.twist.twist.linear.y,current_odom.twist.twist.linear.z;
    a0<< 0.,0.,0.;
    pf<< msg->pose.position.x,msg->pose.position.y,2.0;
    vf<< 0.,0.,0.;
    af<< 0.,0.,0.;
    x_goal = msg->pose.position.x;

    double t_double_now = ros::Time::now().toSec();
    double tau = tau_estimate(p0,pf,1.0,.5);
    double yawSpeed = 0.8;
    traj_exp_x.reinit(get_factors(p0[0],v0[0],a0[0],pf[0],vf[0],af[0],tau),tau,t_double_now+(yaw_exp-yaw_now)/yawSpeed+0.1);
    traj_exp_y.reinit(get_factors(p0[1],v0[1],a0[1],pf[1],vf[1],af[1],tau),tau,t_double_now+(yaw_exp-yaw_now)/yawSpeed+0.1);
    traj_exp_z.reinit(get_factors(p0[2],v0[2],a0[2],pf[2],vf[2],af[2],tau),tau,t_double_now+(yaw_exp-yaw_now)/yawSpeed+0.1);

    std::vector<double> f_yaw;
    f_yaw.push_back(yawSpeed);
    f_yaw.push_back(yaw_now);
    traj_exp_yaw.reinit(f_yaw,(yaw_exp-yaw_now)/yawSpeed,t_double_now);

    std::cout<<"POSITION: "<< p0 <<"---\n"<<pf<<std::endl;
}

void ctrl_pub_step(const ros::TimerEvent&){
    /*定频发布*/
    mavros_msgs::PositionTarget traj_msg;
    double t_double_now = ros::Time::now().toSec();
    PolyTraj traj_exp_vx = traj_exp_x.get_derivate();
    PolyTraj traj_exp_vy = traj_exp_y.get_derivate();
    PolyTraj traj_exp_vz = traj_exp_z.get_derivate();
    PolyTraj traj_exp_ax = traj_exp_vx.get_derivate();
    PolyTraj traj_exp_ay = traj_exp_vy.get_derivate();
    PolyTraj traj_exp_az = traj_exp_vz.get_derivate();
    switch (traj_exp_x.traj_state(t_double_now)) {
        case TRAJ_BEFORE:
            traj_msg.position.x = current_odom.pose.pose.position.x;
            traj_msg.position.y = current_odom.pose.pose.position.y;
            traj_msg.position.z = current_odom.pose.pose.position.z;
            traj_msg.velocity.x = 0.;
            traj_msg.velocity.y = 0.;
            traj_msg.velocity.z = 0.;
            traj_msg.type_mask = traj_msg.IGNORE_YAW;
            break;
        case TRAJ_INCOMMING:
            traj_msg.position.x = traj_exp_x.get_value(t_double_now);
            traj_msg.position.y = traj_exp_y.get_value(t_double_now);
            traj_msg.position.z = traj_exp_z.get_value(t_double_now);
            traj_msg.velocity.x = traj_exp_vx.get_value(t_double_now);
            traj_msg.velocity.y = traj_exp_vy.get_value(t_double_now);
            traj_msg.velocity.z = traj_exp_vz.get_value(t_double_now);
            traj_msg.acceleration_or_force.x = traj_exp_ax.get_value(t_double_now);
            traj_msg.acceleration_or_force.y = traj_exp_ay.get_value(t_double_now);
            traj_msg.acceleration_or_force.z = traj_exp_az.get_value(t_double_now);
            std::cout<<"[b]ERR: "<<traj_msg.position.x-current_odom.pose.pose.position.x
                     <<"  X_GOAL: "<<x_goal<<std::endl;
            break;
        case TRAJ_FINISHED:
            traj_msg.position.x = traj_exp_x.get_value(traj_exp_x.tau_+traj_exp_x.t0_);
            traj_msg.position.y = traj_exp_y.get_value(traj_exp_y.tau_+traj_exp_y.t0_);
            traj_msg.position.z = traj_exp_z.get_value(traj_exp_z.tau_+traj_exp_z.t0_);
            traj_msg.velocity.x = 0.;
            traj_msg.velocity.y = 0.;
            traj_msg.velocity.z = 0.;
            traj_msg.acceleration_or_force.x = 0;
            traj_msg.acceleration_or_force.y = 0;
            traj_msg.acceleration_or_force.z = 0;
            std::cout<<"[c]ERR: "<<traj_msg.position.x - current_odom.pose.pose.position.x
                     <<"  X_GOAL: "<<x_goal<<std::endl;
            break;
    }

    switch (traj_exp_yaw.traj_state(t_double_now)) {
        case TRAJ_BEFORE:
            traj_msg.yaw_rate = 0;
            traj_msg.type_mask = traj_msg.IGNORE_YAW;
            break;
        case TRAJ_INCOMMING:
            traj_msg.yaw = traj_exp_yaw.get_value(t_double_now);
            break;
        case TRAJ_FINISHED:
            traj_msg.yaw = traj_exp_yaw.get_value(traj_exp_yaw.tau_);
            break;
    }

//    if (traj_exp_x.traj_state(t_double_now) == TRAJ_BEFORE)
//        /*当前没有指定轨迹、则悬停在当前点*/
//    {
//        traj_msg.position.x = current_odom.pose.pose.position.x;
//        traj_msg.position.y = current_odom.pose.pose.position.y;
//        traj_msg.position.z = current_odom.pose.pose.position.z;
//        traj_msg.velocity.x = 0.;
//        traj_msg.velocity.y = 0.;
//        traj_msg.velocity.z = 0.;
//        traj_msg.type_mask = traj_msg.IGNORE_YAW;
//        traj_msg.velocity.x = 0.;
//        traj_msg.velocity.y = 0.;
//        traj_msg.velocity.z = 0.;
//        traj_msg.acceleration_or_force.x = 0. ;
//        traj_msg.acceleration_or_force.y = 0. ;
//        traj_msg.acceleration_or_force.z = 0. ;
//        traj_msg.yaw_rate =0. ;
//    }
//    else{
//        traj_msg.position.x = traj_exp_x.get_value(t_double_now);
//        traj_msg.position.y = traj_exp_y.get_value(t_double_now);
//        traj_msg.position.z = traj_exp_z.get_value(t_double_now);
//        traj_msg.velocity.x = traj_exp_vx.get_value(t_double_now);
//        traj_msg.velocity.y = traj_exp_vy.get_value(t_double_now);
//        traj_msg.velocity.z = traj_exp_vz.get_value(t_double_now);
//        traj_msg.acceleration_or_force.x = traj_exp_ax.get_value(t_double_now);
//        traj_msg.acceleration_or_force.y = traj_exp_ay.get_value(t_double_now);
//        traj_msg.acceleration_or_force.z = traj_exp_az.get_value(t_double_now);
//    }
    pose_pub_.publish(traj_msg);
//    std::cout<<"X_STATE: "<<traj_exp_x.traj_state(t_double_now)<<"  YAW_STATE: "<<traj_exp_yaw.traj_state(t_double_now)<<std::endl;
}

mavros_msgs::PositionTarget get_traj(double t)
{
    double r = 1.0, T = 10, thi = (2*3.1416)/T;
    mavros_msgs::PositionTarget res_msg;
    res_msg.position.x = r* cos(thi*t);
    res_msg.position.y = r* sin(thi*t);
    res_msg.position.z = 2.0;
    res_msg.velocity.x = -r*thi*sin(thi*t);
    res_msg.velocity.y = r*thi*cos(thi*t);
    res_msg.velocity.z = 0.0;
    res_msg.acceleration_or_force.x = -r*thi*thi* cos(thi*t);
    res_msg.acceleration_or_force.y = -r*thi*thi* sin(thi*t);
    res_msg.acceleration_or_force.z = 0.0;
    res_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    res_msg.yaw = 0;
    return res_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber order_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal",10,set_pt_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, odom_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    pose_pub_ = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local",10);

    ros::Timer ctrl_pub_loop = nh.createTimer(ros::Duration(0.02),ctrl_pub_step);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
    ros::Rate ctr_rate(2.0);
    ros::Duration hover_dur(3);

    // wait for FCU connection
//    while(ros::ok() && !current_state.connected){
//        ros::spinOnce();
//        rate.sleep();
//    }
//    geometry_msgs::PoseStamped pose;
//    pose.pose.position.x = 0;
//    pose.pose.position.y = 0;
//    pose.pose.position.z = 2;

    //send a few setpoints before starting
//    for(int i = 100; ros::ok() && i > 0; --i){
//        local_pos_pub.publish(pose);
//        ros::spinOnce();
//        rate.sleep();
//    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now(),t0 = last_request;
    int cnt_ = 0;

//    while (ros::ok() && ros::Time::now() - t0 < ros::Duration(3.0))
//    {
//        pose.pose.position.x = 0.0;
//        pose.pose.position.y = 0.0;
//        pose.pose.position.z = 1.5;
//        local_pos_pub.publish(pose);
//    }

    t0 = ros::Time::now();

    mavros_msgs::PositionTarget p_t;
    ROS_INFO("Offboard Publish Startx");
    while(ros::ok()){
//        if(current_state.mode){
//
//        }

//        if( current_state.mode != "OFFBOARD" &&
//            (ros::Time::now() - last_request > ros::Duration(5.0))){
//            if( set_mode_client.call(offb_set_mode) &&
//                offb_set_mode.response.mode_sent){
//                ROS_INFO("Offboard enabled");
//            }
//            last_request = ros::Time::now();
//        } else {
//            if( !current_state.armed &&
//                (ros::Time::now() - last_request > ros::Duration(5.0))){
//                if( arming_client.call(arm_cmd) &&
//                    arm_cmd.response.success){
//                    ROS_INFO("Vehicle armed");
//                }
//                last_request = ros::Time::now();
//            }
//        }

//        p_t = get_traj((ros::Time::now()-t0).toSec());
//        pose_pub_.publish(p_t);

//
//
//        if (ros::Time::now() - t0 > ros::Duration(5.0))
//        {
//            cnt_++;
//            t0 = ros::Time::now();
//        }
//        int cnt__ = cnt_ % 4;
//        switch (cnt__) {
//            case 0:
//                ROS_INFO("0");
//                pose.pose.position.x = 2.0;
//                pose.pose.position.y = 0.0;
//                pose.pose.position.z = 3.0;
//                pose.pose.orientation.w = 1.0;
//                break;
//            case 1:
//                ROS_INFO("1");
//                pose.pose.position.x = 2.0;
//                pose.pose.position.y = 2.0;
//                pose.pose.position.z = 3.0;
//                pose.pose.orientation.w = 1.0;
//                break;
//            case 2:
//                ROS_INFO("2");
//                pose.pose.position.x = 0.0;
//                pose.pose.position.y = 2.0;
//                pose.pose.position.z = 3.0;
//                pose.pose.orientation.w = 2.0;
//                break;
//            case 3:
//                ROS_INFO("3");
//                pose.pose.position.x = 0.0;
//                pose.pose.position.y = 0.0;
//                pose.pose.position.z = 3.0;
//                pose.pose.orientation.w = 1.0;
//                break;
//        }

//        hover_dur.sleep();
//        pose.pose.position.x = 0.0;
//        pose.pose.position.y = 0.2;
//        pose.pose.position.z = 3.0;
//        pose.pose.orientation.w = 1.0;
//        local_pos_pub.publish(pose);

        ros::spinOnce();
//        rate.sleep();
    }
    return 0;
}
