#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <Eigen/Core>
#include<iostream>
using namespace std;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("realsense pcl"));
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    cout<<"get"<<endl;
    // 声明存储原始数据与滤波后的数据的点云的格式
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
    cloud1.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*input, *cloud1);

    viewer1->removeAllPointClouds();  // 移除当前所有点云
    viewer1->addPointCloud(cloud1, "realsense pcl");
    viewer1->updatePointCloud(cloud1, "realsense pcl");
    viewer1->spinOnce(0.001);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/d400/depth/color/points",1,cloud_cb);
    ros::spin();
    cout<<"hello"<<endl;
}