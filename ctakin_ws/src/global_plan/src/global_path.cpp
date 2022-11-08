#include<ros/ros.h>
#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<nav_msgs/Path.h>
#include<fstream>

#include "global_plan/astar.h"

using namespace std;
//地图原始信息
int row,col;
float origin_x=-22.5,origin_y=-5.7;
float resolution=0.05;
//ros发布者节点
ros::Publisher path_pub;
ros::Subscriber pose_sub;

//储存接受到的巡航点
vector<Eigen::Vector2d> start_vector;
vector<Eigen::Vector2d> goal_vector;
bool flag=false;
//储存全局路径
vector<Node*> Path;

//运行时记得修改文件路径为自己的
cv::Mat image=cv::imread("/home/bourne/puppy_ws/src/global_plan/jueying.pgm",0);

//订阅巡检路径点，并规划全局路径，发布路径。
void doPose(const nav_msgs::Path::ConstPtr& pose)
{
    start_vector.clear();
    goal_vector.clear();
    cout<<"in the pose sub!"<<endl;
    for(int i=0;i<(int)pose->poses.size()-1;++i)
    {
        int sx=row-(pose->poses[i].pose.position.y-origin_y)/resolution;
        int sy=(pose->poses[i].pose.position.x-origin_x)/resolution;
        Eigen::Vector2d start(sx,sy);
        start_vector.push_back(start);

        int gx=row-(pose->poses[i+1].pose.position.y-origin_y)/resolution;
        int gy=(pose->poses[i+1].pose.position.x-origin_x)/resolution;
        Eigen::Vector2d goal(gx,gy);
        goal_vector.push_back(goal);
    }
        //读取原图像
    row=image.rows;
    col=image.cols;
    //初始化矩阵  分配内存
    int** pmap=(int**)malloc(row*sizeof(int*));
    for(int i=0;i<row;++i)
    {
        pmap[i]=(int*)malloc(col*sizeof(int));
    }

    for(int  y=0;y<image.rows;++y)
    {
        //用cv::mat::ptr 获取图像的行指针
        unsigned char* row_ptr=image.ptr<unsigned char>(y);  //row_ptr是第y行的头指针
        for(int  x=0;x<image.cols;++x)
        {
            unsigned char* data_ptr=&row_ptr[x*image.channels()];  //data_ptr指向访问处的像素数据

                int data=data_ptr[0]; //--  0 黑色障碍物   205  灰色位置区域   254 白色空闲区
                if(data<250)
                {
                    pmap[y][x]=1;
                }
                else{
                    pmap[y][x]=0;
            }
        }
    }
    int sizex=row;  //502
    int sizey=col;   //733
    //搜索全局路径
    astar2d astar(pmap,sizex,sizey);
    vector<Node*> path;
    Path.clear();
    for(int i=0;i<(int)start_vector.size();++i)
    {
        cout<<"start search!"<<endl;
        astar.reset();
        astar.search(start_vector[i],goal_vector[i]); 
        path=astar.path_nodes;
        for(int i=0;i<(int)path.size();++i)
        {
            Path.push_back(path[i]);
        }
    }
    //发布路径
    cout<<"int the pathpub!"<<endl;
    cout<<Path.size()<<endl;
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    for(int i = 0; i<(int)Path.size();++i)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = Path[i]->position[1]*resolution+origin_x;
        pose_stamped.pose.position.y = (502-Path[i]->position[0])*resolution+origin_y;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        path_msg.poses.push_back(pose_stamped);
    }
    path_pub.publish(path_msg);
    Path.clear();
}

int main(int argc, char *argv[])
{
    cout<<"hello!"<<endl;
    ros::init(argc,argv,"global_path");
    ros::NodeHandle nh("~");
     //定义一个巡航点接受节点
    pose_sub=nh.subscribe<nav_msgs::Path>("/plan_points",10,doPose);
    //全局路径发布节点
    path_pub=nh.advertise<nav_msgs::Path>("/global_path",10);

    ros::spin();
    return 0;
}
