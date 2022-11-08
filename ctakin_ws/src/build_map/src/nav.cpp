#include "build_map/mymap.h"
using namespace std;


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
    for(int i = 0; i< path.size();i++)
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

void print_path(vector<Point*> path)
{
    for(int i=0;i<(int)path.size();i++)
    {
        cout<<"pt "<<i<<" :"<<path[i]->x<<" "<<path[i]->y<<endl;
    }
}

/*相机和odom的同步回调*/
void depthOdomCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImageConstPtr& img )
{
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
    

    //step2.解码相机消息
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    md.depth_image_ = cv_ptr->image;
    if(md.has_image==false) md.has_image=true;

    //step3 判断是否到达目标点
    double dist = pow(pow(md.Pose(0)-goalx,2) + pow(md.Pose(1)-goaly,2),0.5);
    if (dist < 0.2)
    {
        geometry_msgs::Twist speed;
        double v = 0;
        double w = 0;
        speed.linear.x = v;
        speed.angular.z = w;
        cmd_pub_.publish(speed);
        cout<<"到达目标点！"<<endl;
        emit_cmd = false;
        return;
    }
}

inline Eigen::Vector3d cal_point(int u, int v, double depth)
{
    
    Eigen::Vector3d proj_pt;
	proj_pt(1) = -1*(u - mp.cx) * depth / mp.fx;
    proj_pt(2) = -1*(v - mp.cy) * depth / mp.fy;
    proj_pt(0) = depth;
    
    proj_pt = md.Rotation*proj_pt + md.Pose;
    
    return proj_pt;
}


void projectDepth()
{
        ros::Time t1,t2;
	    t1 = ros::Time::now();
        if(!md.has_image) return;
        for(int u = 0;u<640;u=u+3)
        {
            for(int v=0;v<480;v=v+3)
            {
                        double scale = 1/((abs(u-320)/640) +( abs(v-240)/480)+1);
                        scale = 1;
                        double depth = (double)(md.depth_image_.at<uint16_t>(v,u))/1000.0;
                        Eigen::Vector3d pos = cal_point(u,v,depth);
                        if (depth < 0.3 || depth > 4.0) continue;

                        //起始x,y,z
                        double sx = md.Pose(0), sy = md.Pose(1), sz=md.Pose(2);
                        GridIndex start_index = ConvertWorld2GridIndex(sx, sy, sz);
                        //终点x,y,z
                        double gx = pos(0), gy = pos(1), gz = pos(2);
                        GridIndex goal_index = ConvertWorld2GridIndex(gx,gy,gz);

                        GridIndex occ_index;
                        occ_index.SetIndex(goal_index.x,goal_index.y,goal_index.z);
                        //超出范围不处理
                        if (!isValidGridIndex(occ_index))
                        {
                                //cout<<"up boud"<<endl;
                                continue;
                        }
                        
                        int  occ_idx = GridIndexToLinearIndex(occ_index);

                        pMap[occ_idx] = pMap[occ_idx] + 2*scale;
                        if (pMap[occ_idx] >= 100)
                        {
                            pMap[occ_idx] = 100;
                        }

                        auto points = raycast(start_index.x,start_index.y,start_index.z,goal_index.x,goal_index.y,goal_index.z);
                        for(int i=0; i<(int)points.size();i++)
                        {
                                GridIndex free_index;
                                free_index.SetIndex(points[i].x,points[i].y,points[i].z);
                                int idx = GridIndexToLinearIndex(free_index);
                                pMap[idx] = pMap[idx] - 1*scale;
                                if (pMap[idx] <= 10)
                                {
                                    pMap[idx] = 10;
                                }
                        }
                        
            }
        }
    t2 = ros::Time::now();
	//cout<<"frequence: "<<1.0/(t2 - t1).toSec()<<endl;
}

void updateMap(const ros::TimerEvent& event)
{
    //cout<<"update"<<endl;
    //step1 将深度图转为raycast的终点
    projectDepth();
}



void publishMap(const ros::TimerEvent& event)
{
    pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int i=0;i<mp.sizex*mp.sizey*mp.sizez;i++)
    {
        if (pMap[i]>90)
        {
            //cout<<i<<endl;
            //cout<<"p value: "<<(int)pMap[i]<<endl;

            	Eigen::Vector3d pos;
				pos = indexToPos(i);

				//高度大于某个值也不发布
				if(pos(2)>4.0) continue;

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = pos(2);
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

void publish_path(const ros::TimerEvent& event)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "t265_odom_frame";

    for(int i=0;i<(int)path.size();i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        Eigen::Vector3d pos = ConvertGridindex2World(path[i]->x, path[i]->y, -1);
        pose_stamped.pose.position.x = pos(0);
        pose_stamped.pose.position.y = pos(1);

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        path_msg.poses.push_back(pose_stamped);
    }

    path_pub_.publish(path_msg);
}

void cal_cmd(double & vel, double & w)
{
    vel = 0.08;

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
        cout<<"less than 0.2"<<endl;
        cout<<"rth: "<<rth<<endl;
        cout<<"mth: "<<md.theat<<endl;
        double dth = rth - md.theat;
        if(dth > 3.1415)
        {
            dth = dth - 3.1415*2;
        }
        if(dth < -3.1415)
        {
            dth =  dth + 3.1415*2;
        }
        w = 0.2*dth;
        cout<<"--w--"<<w<<endl;
        cout<<"dth: "<<dth<<endl;
    }
else
    {
        cout<<"large than 0.2  ---出现剧烈摆动---"<<endl;
        cout<<"rth: "<<atan2(ry-y,rx-x)<<endl;
        cout<<"mth: "<<md.theat<<endl;
        double dth = atan2(ry-y,rx-x)-md.theat;
        if(dth > 3.1415)
        {
            dth = dth - 3.1415*2;
        }
        if(dth < -3.1415)  
        {
            dth =  dth + 3.1415*2;
        }
        w = 0.3*dth;
        cout<<"--w--"<<w<<endl;
        cout<<"dth: "<<dth<<endl;
    }

    //设定yuzhi
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
    Astar astar(startPos,endPos,pMap,mp.sizex,mp.sizey,mp.sizez); //z设置为20，也就是2米高度内没有障碍物
    for(int i=goal_idx; i<route.size();i++)
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
    GridIndex start_index = ConvertWorld2GridIndex(md.Pose(0),md.Pose(1),md.Pose(2));
    GridIndex goal_index = ConvertWorld2GridIndex(x,y,z);
    Node* startPos  = new Node(start_index.x,start_index.y);
    Node* endPos    = new Node(goal_index.x,goal_index.y);
    Astar astar(startPos,endPos,pMap,mp.sizex,mp.sizey,mp.sizez); //z设置为20，也就是2米高度内没有障碍物
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


void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
    md.Pose(0) = odom->pose.pose.position.x;
    md.Pose(1) = odom->pose.pose.position.y;
    md.Pose(2) = odom->pose.pose.position.z;
    md.Rotation = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation,q);

    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);


}

int main(int argc, char** argv)
{
    goalx = -100;
    goaly = -100;
    init();
    emit_cmd = false;

    ros::init(argc,argv,"build_map");
	ros::NodeHandle nh("~");

    //同步订阅位姿和深度图
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"/t265/odom/sample",1000);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/d400/depth/image_rect_raw",1000);

	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub,depth_sub);
	sync.registerCallback(boost::bind(&depthOdomCallback, _1, _2));

    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/my_map/occupancy", 10);
    path_pub_ = nh.advertise<nav_msgs::Path>("/planning/trajectory", 10);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //计时器1 更新占用地图
    ros::Timer update_timer_ = nh.createTimer(ros::Duration(0.075), updateMap);
    //计时器2 发布地图
    ros::Timer publish_timer_ = nh.createTimer(ros::Duration(0.2), publishMap);
    //计时器3 发布路径
    ros::Timer path_timer_ = nh.createTimer(ros::Duration(0.5), publish_path);
    //计时器4 发布小车控制
    ros::Timer cmd_timer_ = nh.createTimer(ros::Duration(0.1),publish_cmd);
    //计时器5 检测路径过程中的障碍物
    ros::Timer replan_timer_ = nh.createTimer(ros::Duration(0.5),replan);

    //订阅目标点消息
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal",1000, goalCallback);

    //订阅odom消息进行转化
    ros::Subscriber tt = nh.subscribe("/t265/odom/sample",1000, odomCallback);

    ros::spin();
    cout<<"hello"<<endl;
}