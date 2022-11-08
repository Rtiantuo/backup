#include <iostream>
#include <queue>
#include <tuple>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main(int argc,char** argv)
{
	ros::init(argc, argv, "publish_data");
	ros::NodeHandle node("~");
	ros::Publisher pub = node.advertise<sensor_msgs::PointCloud2>("/my_map/occupancy", 10);

  	pcl::PointXYZ pt;
  	pcl::PointCloud<pcl::PointXYZ> cloud;
	double res = 0.1;
	for (int x = 1.5*10; x < 3.5*10; x++)
	{
		for(int y = 1.5*10; y < 3.5*10; y++)
		{
			for(int z = 0; z < 5*10; z++)
			{
				pt.x = x*res;
				pt.y = y*res;
				pt.z = z*res;
				cloud.push_back(pt);				
			}
		}
	}

	for (int x = 1.5*10; x < 3.5*10; x++)
	{
		for(int y = 6.5*10; y < 8.5*10; y++)
		{
			for(int z = 0; z < 5*10; z++)
			{
				pt.x = x*res;
				pt.y = y*res;
				pt.z = z*res;
				cloud.push_back(pt);				
			}
		}
	}


	for (int x = 6.5*10; x < 8.5*10; x++)
	{
		for(int y = 1.5*10; y < 3.5*10; y++)
		{
			for(int z = 0; z < 5*10; z++)
			{
				pt.x = x*res;
				pt.y = y*res;
				pt.z = z*res;
				cloud.push_back(pt);				
			}
		}
	}


	for (int x = 6.5*10; x < 8.5*10; x++)
	{
		for(int y = 6.5*10; y < 8.5*10; y++)
		{
			for(int z = 0; z < 5*10; z++)
			{
				pt.x = x*res;
				pt.y = y*res;
				pt.z = z*res;
				cloud.push_back(pt);				
			}
		}
	}
	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = "map";

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);

	ros::Rate r(1);

	while(ros::ok())
	{
		pub.publish(cloud_msg);
		r.sleep();
		cout<<"published"<<endl;		
	}
}
