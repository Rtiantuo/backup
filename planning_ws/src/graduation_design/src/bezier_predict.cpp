#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include "time_unified.h"

#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace Eigen;

vector<Vector2d> dypoints;
vector<double> beziertime;

// 假设只有一个障碍物（跟丢的话？
void gradient_descent()
{
    

}

// 回调函数，用来塞数据
void dynamic_predict_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{

    // 时间同步 预测的这一小段时间 是从0 开始 往后再相加

    beziertime.push_back(msg->header.stamp.toSec());
    Vector2d pt(msg->pose.position.x,msg->pose.position.y);
    dypoints.push_back(pt);



}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"bezier_predict");
    ros::NodeHandle n;

    ros::Subscriber sub =  n.subscribe("/my_map/dynamic_pose",1,dynamic_predict_callback);

    ros::spin();
    return 0;

}