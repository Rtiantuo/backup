#include <iostream>
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include"pcl/visualization/pcl_visualizer.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "time.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/passthrough.h>
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
	std::cout << "Loaded "  << cloud->width * cloud->height  << " data points from test_pcd.pcd with the following fields: " << std::endl;
        
	//filtered
	pcl::VoxelGrid<pcl::PointXYZ> vox;
	pcl::PointCloud<pcl::PointXYZ>::Ptr vox_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.025f, 0.025f, 0.025f);
	vox.filter(*vox_cloud);
	std::cout <<"PointCloud after filtering has::"<< vox_cloud->points.size()<< "data points" <<std::endl;

	//zhitong
	pcl::PassThrough<pcl::PointXYZ> pass;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rad_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pass.setInputCloud(vox_cloud);	
	pass.setFilterFieldName("z");           
	pass.setFilterLimits(0, 2);    
  	pass.setFilterFieldName("y");           
	pass.setFilterLimits(-1.6, 0.7);    
	pass.setFilterFieldName("x");           
	pass.setFilterLimits(-1.6, 1.5);   
	pass.setFilterLimitsNegative(false);     
	pass.filter(*rad_cloud);

		 //欧式聚类
	std::vector<pcl::PointIndices> ece_inlier;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	tree->setInputCloud(rad_cloud);
	ec.setClusterTolerance(0.08); 
	ec.setMinClusterSize(100);   
	ec.setMaxClusterSize(999999);  
	ec.setSearchMethod(tree); 
	ec.setInputCloud(rad_cloud);  
	ec.extract(ece_inlier);

	//欧式聚类显示
	pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Result of EuclideanCluster"));
	int j=0;
	for(int i=0;i<ece_inlier.size();i++){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> ece_inlier_ext = ece_inlier[i].indices;
		copyPointCloud(*rad_cloud, ece_inlier_ext, *cloud_copy);
		stringstream ss1;
		ss1 <<"EuclideanCluster_clouds" << j<<".pcd";
		//pcl::io::savePCDFileASCII(ss1.str(), *ext_cloud);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>colour2(cloud_copy, color_bar[j][0],color_bar[j][1],color_bar[j][2]);
		viewer2->addPointCloud(cloud_copy, colour2,ss1.str());
		j++;
	}
	viewer2->spinOnce(1000);
		/*while (!viewer2->wasStopped())
	{
		viewer2->spinOnce(100);//刷新
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/

	
}

int main(int argc, char *argv[])
{
    ros::init (argc,argv,"msg_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe<sensor_msgs::PointCloud2>("/d400/depth/color/points",10,doMapmsgs);
    ros::spin();
    return 0;
}
