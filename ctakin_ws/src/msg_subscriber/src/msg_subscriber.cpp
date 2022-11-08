#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include  "iostream"
#include "fstream"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "Eigen/Core"

using namespace std;

void doPclmsgs(const sensor_msgs::PointCloud2::ConstPtr& pcl){
    cout<<"subscriber pcl!"<<endl;
}

int main(int argc, char *argv[])
{
    ros::init (argc,argv,"msg_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub2=nh.subscribe<sensor_msgs::PointCloud2>("/d400/depth/color/points",10,doPclmsgs);

    ros::spin();
    return 0;
}
