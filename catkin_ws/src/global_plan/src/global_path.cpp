#include<ros/ros.h>
#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<nav_msgs/Path.h>
#include<fstream>
#include<nav_msgs/OccupancyGrid.h>

#include "global_path/astar.h"

using namespace std;

//读取的地图的长宽、地图原点、分辨率
int row=502,col=733;
float origin_x=-22.5,origin_y=-5.7;
float resolution=0.05;
//定义一个ros发布者
ros::Publisher path_pub;
//定义/map订阅者话题(占据栅格地图值  // -1 未知  0 空闲  100  占据)
ros::Subscriber map_sub;
ros::Subscriber pose_sub;
//用来储存全局路径
vector<Node*> Path;
//储存接受到的巡航点
vector<Eigen::Vector2d> start_vector;
vector<Eigen::Vector2d> goal_vector;
//通过opencv读取地图
cv::Mat image=cv::imread("/home/tt/dog_ws/src/global_plan/jueying.pgm",0);
string  file_name1="/home/tt/dog_ws/src/global_plan/src/path.csv";
cv::Mat B=cv::Mat(row,col,CV_8UC1);
//将规划得到的全局路径在地图上显示

bool flag=false;
void plot_path()
{
    cout<<"Path size: "<<Path.size()<<endl;
    //存放path在地图上的索引值
    vector<int> path_idex;
    for(int i=0;i<(int)Path.size();++i)
    {
        int px=Path[i]->position(0);
        int py=Path[i]->position(1);
        int index=px*col+py;
        path_idex.push_back(index);
    }
    for(int i=0;i<(int)path_idex.size();++i)
    {
        image.data[path_idex[i]]=0;
    }
    cout<<"all ok!"<<endl;
    return ;
}

//发布规划得到的全局路径
void publish_path(const ros::TimerEvent&event)
{
    cout<<"int the path pub!"<<endl;
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "";

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
    cout<<"push_back  point!!"<<endl;
    path_pub.publish(path_msg);
}

void doMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    cout<<"in the domap!"<<endl;
    col=map->info.width;
    row=map->info.height;
    resolution=map->info.resolution;
    cv::Mat A=cv::Mat(row,col,CV_8UC1);
    for(int i=0;i<map->data.size();++i)
    {
        if(map->data[i]<0 || map->data[i]==100)  //未知和有障碍物的都视为有障碍物
        {
            A.data[i]=1;
        }
        else
        {
            A.data[i]=0;
        }
    }
    cv::flip(A,B,0);
    row=B.rows;
    col=B.cols;
    //初始化二维矩阵  动态分配内存  row*col大小
    int** pmap=(int**)malloc(row*sizeof(int*));
    for(int i=0;i<row;++i)
    {
        pmap[i]=(int*)malloc(col*sizeof(int));
    }

    //将读取进来的image地图的信息放到上面定义好的矩阵中，用于A star的路径搜索
    for(int  x=0;x<B.rows;++x)
    {
        //用cv::mat::ptr 获取图像的行指针
        unsigned char* row_ptr=B.ptr<unsigned char>(x);  //row_ptr是第y行的头指针
        for(int  y=0;y<B.cols;++y)
        {
            unsigned char* data_ptr=&row_ptr[y*B.channels()];  //data_ptr指向访问处的像素数据

                //int data=data_ptr[0]; //--  0 黑色障碍物   205  灰色位置区域   254 白色空闲区 
                pmap[x][y]=data_ptr[0];
        }
    }
    int sizex=row;  //502
    int sizey=col;   //733
    astar2d astar(pmap,sizex,sizey);  //构造函数
    vector<Node*> path;

    for(int i=0;i<(int)start_vector.size();++i)
    {
        cout<<"start_vector.size()"<<start_vector.size()<<endl;
        astar.reset();
	//调用A star来规划路径
    //cout<<"int the path sezrch!"<<endl;
        astar.search(start_vector[i],goal_vector[i]); 
        cout<<start_vector[i]<<endl;
        path=astar.path_nodes;
        for(int i=0;i<(int)path.size();++i)
        {
            Path.push_back(path[i]);
        }
    }
    //plot_path();
    //显示全局路径的地图  "cv_map"是显示窗口的名称
    // cv::imshow("cv_map",image);
    // cv::waitKey();
}

void doPose(const nav_msgs::Path::ConstPtr& pose)
{
    cout<<"in the pose sub!"<<endl;
    for(int i=0;i<(int)pose->poses.size()-1;++i)
    {
        int sx=row-(pose->poses[i].pose.position.y-origin_y)/resolution;
        int sy=(pose->poses[i].pose.position.x-origin_x)/resolution;
        Eigen::Vector2d start(sx,sy);
        //cout<<"start pose: "<<sx<<"  "<<sy<< endl;
        start_vector.push_back(start);

        int gx=row-(pose->poses[i+1].pose.position.y-origin_y)/resolution;
        int gy=(pose->poses[i+1].pose.position.x-origin_x)/resolution;
        Eigen::Vector2d goal(gx,gy);
        goal_vector.push_back(goal);
        //cout<<"goal pose: "<<gx<<"  "<<gy<<endl;
    }
    flag=true;
}

int main(int argc, char *argv[])
{
    //显示原图像
	//cv::imshow("map",image);
    //初始化ros节点句柄
    ros::init(argc,argv,"global_path");
    ros::NodeHandle nh("~");

    //定义一个巡航点接受节点
    pose_sub=nh.subscribe<nav_msgs::Path>("/pose",10,doPose);
    //定义一个ros订阅者  /map
    map_sub=nh.subscribe<nav_msgs::OccupancyGrid>("/map",1,doMap);

//定义一个ros发布者  发布的消息类型为 nav_msgs::Path  发布者的话题名称为/global_path
    //path_pub=nh.advertise<nav_msgs::Path>("/global_path",10);
	//发布规划得到的path  每1s 发布一次
   //ros::Timer path_timer=nh.createTimer(ros::Duration(2),publish_path);
    ros::spin();
    return 0;
}
