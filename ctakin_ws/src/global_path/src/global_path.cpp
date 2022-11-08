#include<ros/ros.h>
#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<nav_msgs/Path.h>

#include "global_path/astar.h"

using namespace std;

int row,col;
float origin_x=-22.5,origin_y=-5.7;
float resolution=0.05;

ros::Publisher path_pub;

vector<Node*> Path;

cv::Mat image=cv::imread("/home/tt/dog_ws/src/global_path/jueying.pgm",0);

// void plot_path()
// {
//     vector<int> path_idex;
//     cout<<" path.size()"<<Path.size()<<endl;
//     for(int i=0;i<(int)Path.size();++i)
//     {
//         int px=Path[i]->position(1);
//         int py=Path[i]->position(0);
//         int index=py*col+px;
//         path_idex.push_back(index);
//     }
//     for(int i=0;i<(int)path_idex.size();++i)
//     {
//         image.data[path_idex[i]]=0;
//     }
//     cout<<"all ok!"<<endl;
//     return ;
// }

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
        cout<<"first path"<<pose_stamped.pose.position.x<<"   "<<  pose_stamped.pose.position.y<<endl;

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        path_msg.poses.push_back(pose_stamped);
    }
    cout<<"push_back  point!!"<<endl;
    path_pub.publish(path_msg);
}

int main(int argc, char *argv[])
{
    cout<<"hello!"<<endl;
    ros::init(argc,argv,"global_path");
    ros::NodeHandle nh("~");
    path_pub=nh.advertise<nav_msgs::Path>("/global_path",10);

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
    cout<<"  sizex"<<sizex<<endl;
    astar2d astar(pmap,sizex,sizey);
    
    Eigen::Vector2d start1(387,450), start2(350,554),start3(290,560),start4(235,520),start5(270,470);
    Eigen::Vector2d goal1(350,554),goal2(290,560),goal3(235,520),goal4(270,470),goal5(387,450);
    vector<Eigen::Vector2d> start_vector{start1,start2,start3,start4,start5};
    vector<Eigen::Vector2d> goal_vector{goal1,goal2,goal3,goal4,goal5};

    vector<Node*> path;

    for(int i=0;i<(int)start_vector.size();++i)
    {
        astar.reset();
        astar.search(start_vector[i],goal_vector[i]); 
        path=astar.path_nodes;
        for(int i=0;i<(int)path.size();++i)
        {
            Path.push_back(path[i]);
        }
    }

    ros::Timer path_timer=nh.createTimer(ros::Duration(10),publish_path);
       
    ros::spin();
    return 0;
}
