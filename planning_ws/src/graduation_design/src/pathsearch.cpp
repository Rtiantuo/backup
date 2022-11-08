#include <ros/ros.h>
#include "rrt.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "drawtree.h"

void addFinalPathMarker(vector<rrtNode> rrtPath)
{   
    finalPath.points.clear();
    geometry_msgs::Point point;
    for(int i = (int)rrtPath.size()-1;i>=0;i--)
    {
        rrtNode tempNode = rrtPath[i];
        point.x = tempNode.x;
        point.y = tempNode.y;
        point.z = 0;
        finalPath.points.push_back(point);
        cout<<"x"<<point.x<<" y"<<point.y<<endl;
    }
    cout<<finalPath.points.size()<<endl;
    treeMarker_pub.publish(finalPath);
}

void goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    RRT RRT;

    //起点
    double x_start = 0;
    double y_start = 0;

    rrtNode rrt_start;
    rrt_start.x = x_start;
    rrt_start.y = y_start;

    // 终点
    double x_goal = msg->pose.position.x;
    double y_goal = msg->pose.position.y;
    rrtNode rrt_goal;
    rrt_goal.x = x_goal;
    rrt_goal.y = y_goal;

    //在rivi中显示起点终点
    initializeMarkers(startPoint, goalPoint, newPoint, rrtTreeMarker, finalPath);
    startPoint.pose.position.x = rrt_start.x;
    startPoint.pose.position.y = rrt_start.y;
    goalPoint.pose.position.x = rrt_goal.x;
    goalPoint.pose.position.y = rrt_goal.y;
    goal_pub.publish(startPoint);
    goal_pub.publish(goalPoint);

    vector<rrtNode> rrtPath = RRT.rrt_search(rrt_start,rrt_goal);

    // 发布最后找到的路径（直线）
    addFinalPathMarker(rrtPath);
    
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"pathsearch");
    ros::NodeHandle n;
    //ros::Subscribe sub1 = n.sunscribe("odom",10,odom_callback);
    ros::Subscriber sub2 = n.subscribe("goal",10,goal_callback);

    goal_pub = n.advertise<visualization_msgs::Marker>("goal_marker",10);
    treeMarker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
    // path_pub = n.advertise<nav_msgs::Path>("/trajectory",1,true);

    ros::spin();
    return 0;
}