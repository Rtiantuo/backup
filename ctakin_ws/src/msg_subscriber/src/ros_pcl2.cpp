#include <iostream>
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include"pcl/visualization/pcl_visualizer.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "time.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
//#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

int color_bar[][3] =
{
	{ 255,0,0},//红色
	{ 0,255,0 },//绿色
	{ 0,0,255 },//蓝色
	{ 0,255,255 },//青色
	{ 255,255,0 },//黄色
	{ 255,255,255 },//白色
	{ 255,0,255 }//紫色
};


void doMapmsgs(const sensor_msgs::PointCloud2::ConstPtr& pcl){
    static int kk = 0;
    kk = kk + 1;
    if (kk % 30 != 0)
    {
	return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl,*cloud);
	std::cout << "Loaded " 
		<< cloud->width * cloud->height 
		<< " data points from test_pcd.pcd with the following fields: " 
		<< std::endl;
        
	//filtered
	pcl::VoxelGrid<pcl::PointXYZ> vox;
	pcl::PointCloud<pcl::PointXYZ>::Ptr vox_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.1f, 0.1f, 0.1f);
	vox.filter(*vox_cloud);
	std::cout <<"PointCloud after filtering has::"<< vox_cloud->points.size()
		<< "data points" <<std::endl;
   //noise
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ>sor;
	pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	sor.setMeanK(50);
	sor.setInputCloud(vox_cloud);
	sor.setStddevMulThresh(1.0);
	sor.filter(*sor_cloud);
	std::cout <<"PointCloud after removing_noise has::"<< sor_cloud->points.size()
		<< "data points" <<std::endl;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3d view"));
	viewer->addPointCloud(sor_cloud);
	/*int i= sor_cloud->size(),j=0;
	while(sor_cloud->size()>i*0.1){

		stringstream ss;
		ss << "sor_clouds" << j <<".pcd";
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> coloud_in_color_h(sor_cloud,color_bar[j][0],color_bar[j][1],color_bar[j][2]);
		
		j++;
	}*/
	viewer->spinOnce(10000);
}

int main(int argc, char *argv[])
{
    ros::init (argc,argv,"msg_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe<sensor_msgs::PointCloud2>("/d400/depth/color/points",10,doMapmsgs);
    ros::spin();
    return 0;
}
