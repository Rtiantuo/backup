#ifndef DRAWPATH_H
#define DRAWPATH_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "rrt_star.h"

ros::Publisher path_pub ;
nav_msgs::Path path;
void initpath(nav_msgs::Path &path)
{
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";
}

void drawpath(double time)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = rrtgoal.x;
    this_pose_stamped.pose.position.y = rrtgoal.y;
    this_pose_stamped.pose.position.z = 0;
    this_pose_stamped.pose.orientation.x = 0;
    this_pose_stamped.pose.orientation.y = 0;
    this_pose_stamped.pose.orientation.z = 0;
    this_pose_stamped.pose.orientation.w = 1;
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";

    for(int i =(rrtPath.size()-1);i>0;i--)
    {
        if( time>=rrtPath[i].t &&  time<rrtPath[i-1].t)
        {
            double dtmpt = time - rrtPath[i].t;
            double dt = rrtPath[i-1].t-rrtPath[i].t;
            this_pose_stamped.pose.position.x = rrtPath[i].x + (rrtPath[i-1].x-rrtPath[i].x)*dtmpt/dt;
            this_pose_stamped.pose.position.y = rrtPath[i].y + (rrtPath[i-1].y-rrtPath[i].y)*dtmpt/dt;
        }
    }

    path.poses.push_back(this_pose_stamped);    
    path_pub.publish(path);
}

#endif