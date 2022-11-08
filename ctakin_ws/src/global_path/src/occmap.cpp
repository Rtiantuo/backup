#include<iostream>
#include<occ_map/mymap.h>
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Point
{
        double x,y;

        Point(double x, double y)
        {
            this->x = x;
            this->y = y;
        }

};

vector<Point*> route;
double goalx;
double goaly;
int goal_idx;

/*相机和odom的同步回调*/
void depthOdomCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImageConstPtr& img )
{
}

void cloudOdomCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    cout<<"get cloud"<<endl;
    //step1 获取位姿和旋转
    md.Pose(0) = odom->pose.pose.position.x;
    md.Pose(1) = odom->pose.pose.position.y;
    md.Pose(2) = odom->pose.pose.position.z;
    md.Rotation = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation,q);

    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    md.theat = yaw;
   
    //step2 获取点云,并且进行体素滤波
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    cout<<"size1: "<<cloud->width*cloud->height<<endl;

	pcl::VoxelGrid<pcl::PCLPointCloud2> down_filter;
    float leaf = 0.05;
    down_filter.setLeafSize(leaf, leaf, leaf);
    down_filter.setInputCloud(cloudPtr);
    down_filter.filter(*cloud);
    cout<<"size2: "<<cloud->width*cloud->height<<endl;

    //step3 剔除掉太远的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl::fromPCLPointCloud2 (*cloud, *cloud_xyz);
    pcl::PointXYZ pt;
    for (int i=0;i<(int)cloud_xyz->points.size();i++)
    {
        double x = cloud_xyz->points[i].z;
        double y = -1*cloud_xyz->points[i].x;
        double z= -1*cloud_xyz->points[i].y;
         double dist = x*x + y*y + z*z;
         if (dist >0.25 && dist < 12)
         {
             Eigen::Vector3d proj_pt;
             proj_pt<<x,y,z;
             proj_pt = md.Rotation*proj_pt + md.Pose;
             pt.x = proj_pt(0);
             pt.y = proj_pt(1);
             pt.z = proj_pt(2);
             cloud_out.push_back(pt);
         }
    }
    cout<<"cloud_out: "<<cloud_out.points.size()<<endl;

    //发布最终点云
    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    cloud_out.is_dense = true;   
    cloud_out.header.frame_id = "t265_odom_frame";

    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud_out, cloud_ros);
    cloud_pub_.publish(cloud_ros);
    
    //存储点云
    md.cloud = cloud_out;
    md.has_cloud = true;

    //更新地图
    update_occupancy();
}

//更新地图
void update_occupancy()
{
    //step1 机器人当前点作为初点
    double sx = md.Pose(0), sy = md.Pose(1), sz = md.Pose(2);
    GridIndex start_index = ConvertWorld2GridIndex(sx,sy,sz);

    //step2 遍历cloud的目标节点
    for (int i=0; i<(int)md.cloud.points.size();i++)
    {
        //step2.1 计算目标点
        double gx =  md.cloud.points[i].x;
        double gy = md.cloud.points[i].y;
        double gz = md.cloud.points[i].z;


        double dist =  sqrt((gx-sx)*(gx-sx) + (gy-sy)*(gy-sy)+(gz-sz)*(gz-sz));
        //最大值为10  最小值为5  距离越小值越大 大于4m时取5
        double scale_occ = 10*(1-min(dist/8,0.5));
        
        GridIndex goal_index = ConvertWorld2GridIndex(gx,gy,gz);

        //step2.2 判断目标点是否在有效范围内
        if(!isValidGridIndex(goal_index))
        {
            continue;
        }

        //step2.3 将目标格子改为占据状态
        int occ_idx = GridIndexToLinearIndex(goal_index);
        md.p_map[occ_idx] = min(md.p_map[occ_idx]+scale_occ,100.0);
        //step2.4 计算经过的空闲网格
        auto points = raycast(start_index.x, start_index.y, start_index.z, goal_index.x, goal_index.y, goal_index.z);
        for(int k=0; k<(int) points.size();k++)
        {
            GridIndex free_idx;
            free_idx.SetIndex(points[k].x,points[k].y,points[k].z);
            int idx = GridIndexToLinearIndex(free_idx);
            double scale_free = 10*max(1-k/double(points.size()),0.5); 
            md.p_map[idx] = max(md.p_map[idx]-scale_free,0.0);
        }
    }
    
}

//发布地图
void publishMap(const ros::TimerEvent& event)
{
    pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int i=0;i<mp.sizex*mp.sizey*mp.sizez;i++)
    {
        if (md.p_map[i]>90)
        {
            	Eigen::Vector3d pos;
				pos = indexToPos(i);

				//高度大于某个值也不发布
				if(pos(2)>4.0) continue;

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = pos(2);
                //cout<<"pt: "<<pos<<endl;
                if(pt.z > 1.5) continue;
				cloud.push_back(pt);
        }
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;   
  cloud.header.frame_id = "t265_odom_frame";

    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_pub_.publish(cloud_msg);
}

//路径发布
void publish_path(const ros::TimerEvent& event)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "t265_odom_frame";

    for(int i=0;i<(int)route.size();i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        //Eigen::Vector3d pos = ConvertGridindex2World(path[i]->x, path[i]->y, -1);
        pose_stamped.pose.position.x = route[i]->x;
        pose_stamped.pose.position.y = route[i]->y;

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        path_msg.poses.push_back(pose_stamped);
    }

    path_pub_.publish(path_msg);
}

int near_pt(double px, double py, vector<Point*> path)
{
    double d = 1000;
    int idx = 0;
    for (int i=0;i<(int) path.size()-1;i++)
    {
        double x = path[i]->x;
        double y = path[i]->y;
        double dist = pow(pow(px-x,2)+pow(py-y,2),0.5) - i*0.0025;
        if (dist < d)
        {
            d= dist;
            idx = i;
        }
    }
    return idx;
}

vector<Point*> gene_path(vector<Point*> path)
{
    vector<Point*>  newpath;
    for(int i=0;i<(int)path.size()-1;i++)
    {
        Point* pt1 = path[i];
        Point* pt2 = path[i+1];
        int num = pow((pow((pt2->x - pt1->x),2) + pow((pt2->y-pt1->y),2)),0.5)/0.02+0.05;
        double orx = pt1->x;
        double ory = pt1->y;
        double dx = (pt2->x - pt1->x)/double(num);
        double dy = (pt2->y - pt1->y)/double(num);
        for(int j = 0; j<num; j++)
        {
            double x = orx + dx*j;
            double y = ory + dy*j;
            Point* pt = new Point(x,y);
            newpath.push_back(pt);
        }
    }
    Point* pt = new Point(path[path.size()-1]->x,path[path.size()-1]->y);
    newpath.push_back(pt);
    return newpath;
}

vector<Point*> smooth_path(vector<Point*> path, double wd, double ws)
{
    vector<Point*>  newpath;
    for(int i=0;i<(int)path.size();i++)
    {
        newpath.push_back(new Point(path[i]->x,path[i]->y));
    }

    for(int epoch = 0;epoch < 100; epoch++)
    {
        for(int i=1;i<(int)path.size()-1;i++)
        {
            newpath[i]->x = newpath[i]->x + wd * (path[i]->x - newpath[i]->x) + ws * (newpath[i-1]->x + newpath[i+1]->x-2*newpath[i]->x);
            newpath[i]->y = newpath[i]->y + wd * (path[i]->y - newpath[i]->y) + ws * (newpath[i-1]->y + newpath[i+1]->y-2*newpath[i]->y);
        }
    }
    return newpath;
}

vector<Point*>  node_to_point(vector<Node*> path)
{
    vector<Point*> rt;
    for(int i = 0; i<(int) path.size();i++)
    {
        int idx = path[i]->x;
        int idy = path[i]->y;
        double px = (idx - mp.offset_x)*mp.resolution+mp.origin_x;
        double py = (idy - mp.offset_y)*mp.resolution+mp.origin_y;
        Point* pt = new Point(px,py);
        rt.push_back(pt);
    }
    return rt;
}

void cal_cmd(double & vel, double & w)
{
    //设定速度为0.1
    vel = 0.1;

    //当前x,y
    double x = md.Pose(0);
    double y = md.Pose(1);
    double dist_to_goal = pow(pow(goalx-x,2) + pow(goaly-y,2),0.5);

    //目标x,y
    int idx = 100;
    idx = near_pt(x,y,route);
    goal_idx = idx;
    double rx = route[idx]->x;
    double ry = route[idx]->y;
    double rth = atan2(route[idx+1]->y-ry,route[idx+1]->x-rx);

    //若已经靠近目标点，就将目标点作为目标
    cout<<"-----dis_to_goal----"<<dist_to_goal<<endl;
    if (dist_to_goal < 0.35)
    {

        rx = goalx;
        ry = goaly;
        rth = atan2(goaly-ry,goalx-rx);
    }

    //计算dist
    double dist = pow(pow(rx-x,2) + pow(ry-y,2),0.5);

    //若dist小于某个设定之
    if (dist <= 0.2)
    {
        double dth = rth - md.theat;
        if(dth > 3.1415)
        {
            dth = dth - 3.1415*2;
        }
        if(dth < -3.1415)
        {
            dth =  dth + 3.1415*2;
        }
        w = 0.8*dth;
    }
else
    {
        double dth = atan2(ry-y,rx-x)-md.theat;
        if(dth > 3.1415)
        {
            dth = dth - 3.1415*2;
        }
        if(dth < -3.1415)  
        {
            dth =  dth + 3.1415*2;
        }
        w = 0.8*dth;
    }

    //设定阈值
    if (w>0.5)  w=0.5;
    if (w<-0.5) w=-0.5;

}
//发布控制信息
void publish_cmd(const ros::TimerEvent& event)
{
    if(emit_cmd == false)
    {
        return;
    }
    //step3. 发布小车控制指令
    geometry_msgs::Twist speed;
    double v = 0;
    double w = 0;
    cal_cmd(v,w);

    speed.linear.x = v;
    speed.angular.z = w;
    cmd_pub_.publish(speed);
}

//发布控制信息
void replan(const ros::TimerEvent& event)
{
    if(emit_cmd == false)
    {
        return;
    }
    //查看是否有碰撞
    GridIndex start_index = ConvertWorld2GridIndex(md.Pose(0),md.Pose(1),md.Pose(2));
    GridIndex goal_index = ConvertWorld2GridIndex(goalx,goaly,0);
    Node* startPos  = new Node(start_index.x,start_index.y);
    Node* endPos    = new Node(goal_index.x,goal_index.y);
    Astar astar(startPos,endPos,md.p_map,mp.sizex,mp.sizey,mp.sizez); //z设置为20，也就是2米高度内没有障碍物
    for(int i=goal_idx; i<(int)route.size();i++)
    {
        double x = route[i]->x;
        double y = route[i]->y;

        int idx = std::ceil((x - mp.origin_x) / mp.resolution) + mp.offset_x;
        int idy = std::ceil((y - mp.origin_y) / mp.resolution) + mp.offset_y;

        if(astar.pMap[idx][idy]==1)
        {
                cout<<"replan"<<endl;
                path = astar.search();
                if(path.size()==0) return;
                auto p1 = node_to_point(path);
                auto p2 = smooth_path(p1,0.5,0.1);
                auto p3 = gene_path(p2);
                auto p4 = smooth_path(p3,0.1,0.5);
                route = p4;
                return;
        }
    }
}


/*odom的回调信息*/
void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal)
{
    cout<<"get"<<goal->pose.position.x<<endl;
    double x = goal->pose.position.x;
    double y = goal->pose.position.y;
    double z = 0;
    //--当前相机所在位置
    GridIndex start_index = ConvertWorld2GridIndex(md.Pose(0),md.Pose(1),md.Pose(2));
    //--当前目标点
    GridIndex goal_index = ConvertWorld2GridIndex(x,y,z);
    Node* startPos  = new Node(start_index.x,start_index.y);
    Node* endPos    = new Node(goal_index.x,goal_index.y);
    Astar astar(startPos,endPos,md.p_map,mp.sizex,mp.sizey,mp.sizez); //z设置为20，也就是2米高度内没有障碍物
    path = astar.search();
    if(path.size()==0) return;
    auto p1 = node_to_point(path);
    auto p2 = smooth_path(p1,0.5,0.1);
    auto p3 = gene_path(p2);
    auto p4 = smooth_path(p3,0.1,0.5);
    route = p4;
    goalx = x;
    goaly = y;
     double dist = pow(pow(md.Pose(0)-goalx,2) + pow(md.Pose(1)-goaly,2),0.5);
    if (dist < 0.2)
    {
        emit_cmd = false;
         geometry_msgs::Twist speed;
        double v = 0;
        double w = 0;
        speed.linear.x = v;
        speed.angular.z = w;
        cmd_pub_.publish(speed);
        return;
    }

    emit_cmd = true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"build_map");
	ros::NodeHandle nh("~");

    //初始化
    init();
    emit_cmd=false;

    //订阅 t265位置话题
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"/t265/odom/sample",1000);
    //订阅d455深度话题
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/d400/depth/image_rect_raw",1000);
    //订阅d455点云话题
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh,"/d400/depth/color/points",1000);
    //同步订阅两个话题
	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync1(MySyncPolicy(10),odom_sub,depth_sub);
	sync1.registerCallback(boost::bind(&depthOdomCallback, _1, _2));

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy2;
    Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(10),odom_sub,cloud_sub);
	sync2.registerCallback(boost::bind(&cloudOdomCallback, _1, _2));
    //分别发布地图、处理后的点云、路径话题
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/my_map/pointcloud", 10);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/my_map/occ_map", 10);
    path_pub_ = nh.advertise<nav_msgs::Path>("/planning/trajectory", 10);
    //发布机器人底盘控制话题，测试时需要按照具体的机器人底盘话题名称进行修改。
    cmd_pub_=nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    //定时器发布地图
    ros::Timer publish_timer_ = nh.createTimer(ros::Duration(0.2), publishMap);
     //计时器发布路径
    ros::Timer path_timer_ = nh.createTimer(ros::Duration(0.5), publish_path);
      //计时器发布小车控制
    ros::Timer cmd_timer_ = nh.createTimer(ros::Duration(0.1),publish_cmd);
    //计时器检测路径过程中的障碍物
    ros::Timer replan_timer_ = nh.createTimer(ros::Duration(0.5),replan);

    ros::Subscriber goal_sub_=nh.subscribe("/move_base_simple/goal",1000,goalCallback);

    ros::spin();
    return 0;
}