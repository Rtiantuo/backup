#include "string"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "mavros_msgs/PositionTarget.h"
#include "vector"


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::PositionTarget get_traj(double t)
{
    double r = 20.0, T = 10, thi = (2*3.1416)/T;
    mavros_msgs::PositionTarget res_msg;
    res_msg.position.x = r* cos(thi*t);
    res_msg.position.y = r* sin(thi*t);
    res_msg.position.z = 3;
    res_msg.velocity.x = -r*thi*sin(thi*t);
    res_msg.velocity.y = r*thi*cos(thi*t);
    res_msg.velocity.z = 0;
    res_msg.acceleration_or_force.x = -r*thi*thi* cos(thi*t);
    res_msg.acceleration_or_force.y = -r*thi*thi* sin(thi*t);
    res_msg.acceleration_or_force.z = 0;
    res_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    res_msg.yaw = thi * t + M_PI;
//    res_msg.position.x = t + pow(t,2) + pow(t,3);
//    res_msg.position.y = t + pow(t,2) + pow(t,3);
//    res_msg.position.z = 3+ sin(t/5*3.14);
//    res_msg.velocity.x = 1 + 2*t + 3* pow(t,2);
//    res_msg.velocity.y = 1 + 2*t + 3* pow(t,2);
//    res_msg.velocity.z = 0;
//    res_msg.acceleration_or_force.x = 2 + 3*t;
//    res_msg.acceleration_or_force.y = 2 + 3*t;
//    res_msg.acceleration_or_force.z = 0;
//    res_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
//    res_msg.yaw = sin(t/5*3.14);
    return res_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher pose_pub_ = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local",10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
    ros::Rate ctr_rate(2.0);
    ros::Duration hover_dur(3);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now(),t0 = last_request;
    int cnt_ = 0;

    ROS_INFO("AA");
    while (ros::ok() && ros::Time::now() - t0 < ros::Duration(3.0))
    {
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 2.0;
        local_pos_pub.publish(pose);
    }

    t0 = ros::Time::now();

    mavros_msgs::PositionTarget p_t;
    while(ros::ok()){
        p_t = get_traj((ros::Time::now()-t0).toSec());
        pose_pub_.publish(p_t);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
