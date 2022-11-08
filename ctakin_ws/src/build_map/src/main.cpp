#include <iostream>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<ctime>
#include<vector>
#include<queue>

#include<build_map/raycast.h>

using namespace std;
using namespace message_filters;

int INVALID_IDX = -10000;

#define logit(x) (log((x) / (1 - (x))))

/*地图数据*/
struct MappingData{
	std::vector<double> occupancy_buffer_;
	std::vector<char> occupancy_buffer_neg;
  	std::vector<char> occupancy_buffer_inflate_;

	Eigen::Vector3d camera_pos_;
	Eigen::Quaterniond camera_q_;
	cv::Mat depth_image_;
	int proj_points_cnt;
	vector<Eigen::Vector3d> proj_points_;

	int raycast_num_;

	vector<short> count_hit_, count_hit_and_miss_;
	queue<Eigen::Vector3i> cache_voxel_;
} ;

/*地图参数*/
struct MappingParameters{
	  double k_depth_scaling_factor_;
	  double cx,cy;
	  double fx,fy;

	double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  	double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,min_occupancy_log_;                   // logit of occupancy probability
	double min_ray_length_, max_ray_length_;

	  Eigen::Vector3d map_origin_,map_size_;
	  double resolution_, resolution_inv_;
	  Eigen::Vector3i map_voxel_num_;

	double unknown_flag_;
};

MappingData md_;
MappingParameters mp_;

/*相关参数初始化*/
void init()
{
	mp_.k_depth_scaling_factor_ = 1000;
	mp_.cx = 315.917;
	mp_.cy = 243.442;
	mp_.fx = 387.355;
	mp_.fy = 387.355;

	/*raycast相关*/
	mp_.p_hit_ = 0.65;
	mp_.p_miss_ = 0.35;
	mp_.p_min_ = 0.12;
	mp_.p_max_ = 0.90;
	mp_.p_occ_ = 0.80;



	mp_.min_ray_length_ = 0.5;
	mp_.max_ray_length_ =  5.0;

	double x_size = 10, y_size = 10, z_size = 3, ground_height = -1.0;
	mp_.map_origin_= Eigen::Vector3d(x_size*(-0.5),y_size*(-0.5),ground_height);
	mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
	mp_.resolution_ = 0.1;
	mp_.resolution_inv_ = 1.0/mp_.resolution_;



	for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
	for (int i = 0; i < 3; ++i ) cout<<mp_.map_voxel_num_(i)<<endl;

	int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
 	md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  	md_.count_hit_ = vector<short>(buffer_size, 0);

	md_.proj_points_.resize(640*480);
	md_.proj_points_cnt = 0;
	md_.raycast_num_ = 0;

	mp_.prob_hit_log_ = logit(mp_.p_hit_);
  	mp_.prob_miss_log_ = logit(mp_.p_miss_);
  	mp_.clamp_min_log_ = logit(mp_.p_min_);
  	mp_.clamp_max_log_ = logit(mp_.p_max_);
  	mp_.min_occupancy_log_ = logit(mp_.p_occ_);
	mp_.unknown_flag_ = 0.01;

	md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
	md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  	md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
	
}

inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline int toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int toAddress(int& x, int& y, int& z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

/*depth的回调信息*/
void depthCallback(const sensor_msgs::ImageConstPtr& img) {
  std::cout << "depth: " << img->header.stamp << std::endl;
}

/*odom的回调信息*/
void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
	md_.camera_pos_(0) = odom->pose.pose.position.x;
	md_.camera_pos_(1) = odom->pose.pose.position.y;
	md_.camera_pos_(2) = odom->pose.pose.position.z;
	md_.camera_q_ =  Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
	cout<<"pose: "<<odom->header.stamp <<endl;
}

//投影深度图
void projectDepthImage()
{
	//cout<<"project"<<endl;

	md_.proj_points_cnt = 0;

	uint16_t* row_ptr;
	int  cols = md_.depth_image_.cols;
	int rows = md_.depth_image_.rows;

	double depth;

	Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

	for (int v=0; v<rows; v++,v++,v++,v++)
	{
		row_ptr = md_.depth_image_.ptr<uint16_t>(v);
		for(int u=0;u<cols;u++,u++,u++,u++)
		{
			Eigen::Vector3d proj_pt;
			depth = (*row_ptr++)/mp_.k_depth_scaling_factor_;

			proj_pt(0) = (u - mp_.cx) * depth / mp_.fx;
            proj_pt(1) = (v - mp_.cy) * depth / mp_.fy;
            proj_pt(2) = depth;

            proj_pt = camera_r * proj_pt + md.;
		}
	}
}

int setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
	if (occ !=1 && occ!=0) return INVALID_IDX;

	Eigen::Vector3i id;
	posToIndex(pos,id);
	int idx_ctns = toAddress(id);
	
    md_.count_hit_and_miss_[idx_ctns] += 1;
    
  	if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    	md_.cache_voxel_.push(id);
  	}

  	if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  	return idx_ctns;
}

//通过点云计算射线，确定需要更新的珊格
void  raycastProcess()
{
	if(md_.proj_points_cnt==0) return;

	md_.raycast_num_ += 1;

	int vox_idx;
	double length;

	double min_x = 0;
	double min_y = 0;
	double min_z = 0;

	double max_x = 0;
	double max_y = 0;
	double max_z = 0;

	RayCaster raycaster;
	Eigen::Vector3d half = Eigen::Vector3d(0.5,0.5,0.5);
	Eigen::Vector3d ray_pt,pt_w;

	for(int i=0; i< md_.proj_points_cnt;i++)
	{
		pt_w = md_.proj_points_[i];

		length = (pt_w - md_.camera_pos_).norm();

		if(length > mp_.max_ray_length_)
		{
			pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
			vox_idx = setCacheOccupancy(pt_w,0);
		}
		else
		{
			vox_idx = setCacheOccupancy(pt_w,1);
		}

		raycaster.setInput(pt_w/mp_.resolution_,md_.camera_pos_/mp_.resolution_);
		while(raycaster.step(ray_pt))
		{
			Eigen::Vector3d tmp = (ray_pt+half)*mp_.resolution_;
			length = (tmp-md_.camera_pos_).norm();

			vox_idx = setCacheOccupancy(tmp,0);
		}

		while(!md_.cache_voxel_.empty())
		{
			Eigen::Vector3i idx = md_.cache_voxel_.front();
			int idx_ctns = toAddress(idx);
			md_.cache_voxel_.pop();
			double log_odds_update = mp_.prob_hit_log_;
			if (md_.count_hit_and_miss_[idx_ctns]>=2*md_.count_hit_[idx_ctns])
			{
				log_odds_update = mp_.prob_miss_log_;
			}

			md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    	if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      		continue;
   		 } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      		md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
     		 continue;
    	}

		md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_), mp_.clamp_max_log_);
	}

	}

}

void publishMap(const ros::TimerEvent& event)
{
	cout<<"publish map"<<endl;
	pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	//发布假点云
	int minx = 0, maxx = mp_.map_size_(0)/mp_.resolution_;
	int miny = 0, maxy = mp_.map_size_(1)/mp_.resolution_;
	int minz = 0, maxz = mp_.map_size_(2)/mp_.resolution_;
	cout<<"x: "<<maxx<<" y:"<<maxy<<"z: "<<maxz<<endl;
	cout<<"buff test: "<<md_.occupancy_buffer_[toAddress(minx,miny,minz)]<<endl;
	for(int x=minx;x<maxx;x++)
		for(int y=miny;y<maxy;y++)
			for(int z = minz;z<maxz;z++)
			{
				//空闲点不发布
				if (md_.occupancy_buffer_[toAddress(x,y,z)]==0)
				{
					cout<<"yes"<<endl;
					continue;
				}

				Eigen::Vector3d pos;
				indexToPos(Eigen::Vector3i(x,y,z),pos);

				//高度大于某个值也不发布
				if(pos(2)>5.0) continue;

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = pos(2);
				cloud.push_back(pt);
			}
	
	cout<<"point size: "<<cloud.points.size()<<endl;
}

//更新占用地图
void updateOccupancy() 
{
	ros::Time t1,t2;
	t1 = ros::Time::now();
	//step1. 投影深度图
	projectDepthImage();

	//step2.进行射影变换，并且更新占用地图
	raycastProcess();

	t2 = ros::Time::now();
	//cout<<"frequence: "<<1.0/(t2 - t1).toSec()<<endl;
} 

/*相机和odom的同步回调*/
void depthOdomCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImageConstPtr& img )
{
	//cout<<"odom time step"<<odom->header.stamp <<endl;
	//step1. 解码odom消息
   md_.camera_pos_(0) = odom->pose.pose.position.x;
   md_.camera_pos_(1) = odom->pose.pose.position.y;
   md_.camera_pos_(2) = odom->pose.pose.position.z;
   md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
 
	//step2.解码相机消息
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
    }
   cv_ptr->image.copyTo(md_.depth_image_);

	//step3.更新占用地图
	updateOccupancy();
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"build_map");
	ros::NodeHandle nh("~");

	init();
	//ros::Subscriber  odomsub = nh.subscribe("/t265/odom/sample",1000, odomCallback);
	//ros::Subscriber  depthsub = nh.subscribe("/d400/depth/image_rect_raw",1000, depthCallback);

	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"/t265/odom/sample",1000);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/d400/depth/image_rect_raw",1000);

	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub,depth_sub);
	sync.registerCallback(boost::bind(&depthOdomCallback, _1, _2));

	//低频率发送全局点云
	ros::Timer globalmap_timer_ = nh.createTimer(ros::Duration(1), publishMap);

	map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
	ros::spin();
	cout<<"hello"<<endl;
}
