#include<iostream>
#include<ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include<queue>
#include<unordered_map>
#include<iostream>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Twist.h>

#include<myastar.h>

using namespace std;

int** pmap;
int sizex = 100;
int sizey = 100;
ros::Publisher obs_pub;
ros::Publisher path_pub;
vector<Node*> path;

struct newNode   
{
	double f;
    double g;
    double h;
    int state;
	newNode(double g1, double h1) 
	{
		g = g1;
        h = h1;
        f = g + h;
        state = 0;
	}
};



typedef newNode* newNodePtr;

typedef unordered_map<int, newNodePtr> explored;
explored mymap;

void init_map()
{
    pmap = (int **)malloc(sizex * sizeof(int *));
    for(int i=0; i<sizex;i++)
    {
        pmap[i] = (int *) malloc(sizey*sizeof(int));
    }

    for(int x=0;x<sizex;x++)
    {
        for(int y=0;y<sizey;y++)
        {
            pmap[x][y]=0;
        }
    }

    int width = 13;

    int centx = 25+5;
    int centy = 25+5;
    for(int x=centx-width;x<centx+width;x++)
    {
        for(int y=centy-width;y<centy+width;y++)
        {
            pmap[x][y]=1;
        }
    }

    centx = 25+5;
    centy = 75-5;
    for(int x=centx-width;x<centx+width;x++)
    {
        for(int y=centy-width;y<centy+width;y++)
        {
            pmap[x][y]=1;
        }
    }

    centx = 75+5;
    centy = 25+5;
    for(int x=centx-width;x<centx+width;x++)
    {
        for(int y=centy-width;y<centy+width;y++)
        {
            pmap[x][y]=1;
        }
    }

    centx = 75-5;
    centy = 75-5;
    for(int x=centx-width;x<centx+width;x++)
    {
        for(int y=centy-width;y<centy+width;y++)
        {
            pmap[x][y]=1;
        }
    }
}

void publish_obs(const ros::TimerEvent &event)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int x = 0; x < sizex; x++)
    {
        for(int y=0; y<sizey; y++)
        {
            //高度大于某个值也不发布
            if (pmap[x][y] == 0)
                continue;
            pt.x = x*0.1;
            pt.y = y*0.1;
            pt.z = 0;
            cloud.push_back(pt);
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    obs_pub.publish(cloud_msg);
}

void publish_path(const ros::TimerEvent &event)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    for(int i=0; i<(int)path.size();i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = path[i]->position[0]*0.1;
        pose_stamped.pose.position.y = path[i]->position[1]*0.1;

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        path_msg.poses.push_back(pose_stamped);
    }
    path_pub.publish(path_msg);
}

int main(int argc,char** argv)
{
    cout<<"planner"<<endl;
 
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");

    init_map();

    path_pub = nh.advertise<nav_msgs::Path>("/planning/astar",10);

    myAstar astar(pmap, sizex, sizey);

    Eigen::Vector2d start(15,15), goal(90,90);
    
    auto t1 = ros::Time::now();
    for(int i=0;i<1;i++)
    {
            astar.reset();
    astar.search(start,goal);
    }


    auto t2 = ros::Time::now();
    cout<<"smooth time duration: "<<(t2-t1).toSec()<<endl;

    path = astar.path_nodes_;

    ros::Timer path_timer = nh.createTimer(ros::Duration(0.5),publish_path);

    ros::Timer obs_timer = nh.createTimer(ros::Duration(0.5),publish_obs);
    obs_pub = nh.advertise<sensor_msgs::PointCloud2>("/mymap/map", 1, true);

    ros::spin();
}

    /*

    priority_queue<newNode*,vector<newNode*>,cmp> queue1;
    newNode* newnode1 = new newNode(2,3);
    queue1.push(newnode1);

    newNode* newnode2 = new newNode(3,3);
    queue1.push(newnode2);

    newNode* newnode3= new newNode(4,3);
    queue1.push(newnode3);

    newNode* newnode4 = new newNode(5,3);
    queue1.push(newnode4);


    while (!queue1.empty())
	{
		std::cout << queue1.top()->f <<std::endl;
		queue1.pop();
	}

    newNodePtr node1 = new newNode(2,4);
    mymap[1] = node1;
    newNodePtr node2 = new newNode(3,4);
    mymap[2] = node2;
    newNodePtr node3 = new newNode(4,4);
    mymap[3] = node3;

    mymap[3]->state = 1;
    int x = 3;
    if( mymap.find(x)!=mymap.end() ){//找到key值为2的键值对
        cout<<"state of mymap "<<mymap[x]->state<<endl;
    }

    mymap[1] = node1;
    mymap[2] = node1;
    int x = 2;
    if( mymap.find(x)!=mymap.end() ){//找到key值为2的键值对
        cout<<"get data where key=2! and data="<<mymap[x]<<endl;
    }

	Data_Compare a(2,4);
	Data_Compare b(3,5);
	Data_Compare c(5,2);
    Data_Compare d(5,5);
	priority_queue<Data_Compare> queue1;
	queue1.push(b);
	queue1.push(c);
	queue1.push(a);
    queue1.push(d);

    while (!queue1.empty())
	{
		std::cout << queue1.top().f <<std::endl;
		queue1.pop();
	}*/