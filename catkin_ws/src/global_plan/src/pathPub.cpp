#include<ros/ros.h>
#include<iostream>
#include<nav_msgs/Path.h>

using namespace std;
ros::Publisher path_pub;

void pose_pub(const ros::TimerEvent&event)
{
    cout<<"in the pose pub!"<<endl;
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "";

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x=0.0269626;
    pose.pose.position.y=0.0740975;
    pose.pose.orientation.x=0;
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=0;
    pose.pose.orientation.w=1;
    path_msg.poses.push_back(pose);

    pose.pose.position.x=5.22321;
    pose.pose.position.y=1.93067;
    pose.pose.orientation.x=0;
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=0;
    pose.pose.orientation.w=1;
    path_msg.poses.push_back(pose);
    path_pub.publish(path_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"pathPub");
    ros::NodeHandle nh;
    path_pub=nh.advertise<nav_msgs::Path>("/pose",1);
    ros::Timer pose_timer=nh.createTimer(ros::Duration(1),pose_pub);
    ros::spin();
    return 0;
}
