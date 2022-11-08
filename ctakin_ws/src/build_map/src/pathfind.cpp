#include "build_map/mymap.h"
#include "build_map/Astar.h"
int startx = 50, starty = 10;
int goalx = 90, goaly = 90;

void init_map()
{
    //障碍物信息, 一共10米
    /*for(double x = 1.5;x<8.5;x=x+mp.resolution)
    {
        for(double y=1.5;y<8.5;y=y+mp.resolution)
        {
            if(abs(x-5)<3 && abs(y-5)<3) continue;
            for(double z = 0;z<0.2;z=z+mp.resolution)
            {
                //将pMap的x,y,z位置处设置障碍物信息
                GridIndex occ_index = ConvertWorld2GridIndex(x, y, z);
                int  occ_lineidx = GridIndexToLinearIndex(occ_index);
                pMap[occ_lineidx] = 95;
            }
        }
    }*/
    for(int x=0; x<mp.sizex;x++)
    {
        for(int y=0; y<mp.sizey;y++)
        {
            for(int z=0;z<mp.sizez;z++)
            {
                GridIndex occ_index;
                occ_index.SetIndex(x,y,z);
                int  occ_lineidx = GridIndexToLinearIndex(occ_index);
                if (abs(x-mp.sizex*0.5)<mp.sizex*0.25 && abs(y-mp.sizey*0.5)<mp.sizey*0.25)
                {
                    pMap[occ_lineidx] = 90;
                }
                else
                {
                    pMap[occ_lineidx] = 10;
                }
                
                if (x==startx && y==starty)
                {
                    pMap[occ_lineidx] = 0;
                }
                if (x==goalx && y==goaly)
                {
                    pMap[occ_lineidx] = 0;
                }
            }
        }
    }

}

void Astart(double startx, double starty, double goalx, double goaly)
{

}

void publish_Map(const ros::TimerEvent& event)
{
    cout<<"publish"<<endl;
    pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int i=0;i<mp.sizex*mp.sizey*mp.sizez;i++)
    {
        if (pMap[i]>80)
        {

            	Eigen::Vector3d pos;
				pos = indexToPos(i);

				//高度大于某个值也不发布

				pt.x = pos(0);
				pt.y = pos(1);
				pt.z = pos(2);
                //cout<<"pt: "<<pos(1)<<" "<<pos(2)<<" ";
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
    cout<<endl;
}

void publish_path(const ros::TimerEvent& event)
{
    cout<<"publish path"<<endl;
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

/*odom的回调信息*/
void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal)
{
    cout<<"get"<<goal->pose.position.x<<endl;
    double x = goal->pose.position.x;
    double y = goal->pose.position.y;
    double z = 0;
    GridIndex goal_index = ConvertWorld2GridIndex(x,y,z);
    Node* startPos = new Node(startx,starty);
    Node* endPos = new Node(goal_index.x,goal_index.y);
    Astar astar(startPos,endPos,pMap,mp.sizex,mp.sizey,20); //z设置为20，也就是2米高度内没有障碍物
    path = astar.search();
    cout<<path.size()<<endl;
}

int main(int argc, char** argv)
{
    init();
    init_map();
    ros::init(argc,argv,"find_path");
	ros::NodeHandle nh("~");

    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning/occupancy", 10);
    path_pub_ = nh.advertise<nav_msgs::Path>("/planning/trajectory", 10);
    //计时器2 发布地图
    ros::Timer map_timer_ = nh.createTimer(ros::Duration(2), publish_Map);
    ros::Timer path_timer_ = nh.createTimer(ros::Duration(0.5), publish_path);

    //订阅目标点消息
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal",1000, goalCallback);



    //Astar astar(startPos,endPos,pMap,mp.sizex,mp.sizey,mp.sizez);
 

    ros::spin();
    cout<<"hello"<<endl;
}