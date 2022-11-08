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

void call_back(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
    	pcl::fromROSMsg (*input, cloud);//cloud is the output

	for(int i = 0; i < int(cloud.points.size()); i++)
	{
		cout<<"point "<<i<<"'s cord:"<<endl;
		cout<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<endl;
	}

	cout<<"received"<<endl;
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "receive_data");
	ros::NodeHandle node("~");

	ros::Subscriber sub = node.subscribe("/my_map/occupancy", 1, call_back);

	ros::spin();
}
