#include<ros/ros.h>
#include<iostream>
#include <boost/shared_ptr.hpp>
#include"agg_plan/aggplan.h"
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<Eigen/Eigen>
#include<fstream>
#include<cstdlib>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>

using namespace std;

//读入障碍物信息，放进env_data中  并转换为占据栅格地图
void read_env()
{
    //cout<<"int the env_read!"<<endl;
    ifstream file("/home/tt/planning_ws/src/agg_plan/env1.csv",ios::in);
    if(file.fail())
    {
        cout<<"failed open file!"<<endl;
        exit(1);
    }
    string line;
    while(getline(file,line))
    {
        string field;
        Eigen::Vector3d point;
        istringstream sin(line);
        int i = 0;
        if(line == " ") break;
        while(getline(sin,field,','))
        {
            point(i) = atof(field.c_str());
            ++i;
        }
        Point* pt = new Point(point(0),point(1),point(2));
        env_data.push_back(pt);
    }
    //转换为栅格地图
    for(int i = 0;i<(int)env_data.size();++i)
    {
        int px = floor((env_data[i]->x - minx)/res +0.001);
        int py = floor((env_data[i]->y - miny)/res +0.001);
        int pz = floor((env_data[i]->z - minz )/res +0.001);
        int index = px +py*sizex + pz*sizex*sizey;
        pmap[index] = 100;
    }
    //cout<<"all read!"<<endl;
}

//发布障碍物环境信息
void publish_env(const ros::TimerEvent& event)
{
    //cout<<"int the env_pub!"<<endl;
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloudPtr->width = env_data.size();
    cloudPtr->height = 1;
    cloudPtr->points.resize(cloudPtr->width * cloudPtr->height);
    for(int i = 0;i<(int)env_data.size();++i)
    {
        pt.x = env_data[i]->x;
        pt.y = env_data[i]->y;
        pt.z = env_data[i]->z;
        cloud.push_back(pt);
        cloudPtr->points[i].x = env_data[i]->x;
        cloudPtr->points[i].y = env_data[i]->y;
        cloudPtr->points[i].z = env_data[i]->z;
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";
    //cloudPtr =boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud);
    //点云放进kdtree
    kdTree.setInputCloud(cloudPtr);
    sensor_msgs::PointCloud2 cloud_msgs;
    pcl::toROSMsg(cloud,cloud_msgs);
    map_pub.publish(cloud_msgs);
    //cout<<"ok!"<<endl;
}

//将route数据写入csv文档
void write_route()
{
    ofstream outfile;
    outfile.open("/home/tt/planning_ws/src/agg_plan/route.csv");
    for(int i =0;i<(int)route.size();++i)
    {
        outfile<<route[i]->t<<","<<route[i]->x<<","<<route[i]->y<<","<<route[i]->z<<endl;
    }
    outfile.close();

    ofstream velfile;
    velfile.open("/home/tt/planning_ws/src/agg_plan/vels.csv");
    for(int i =0;i<(int)vels.size();++i)
    {
        velfile<<vels[i]->t<<","<<vels[i]->x<<","<<vels[i]->y<<","<<vels[i]->z<<endl;
    }
    velfile.close();

    ofstream accfile;
    accfile.open("/home/tt/planning_ws/src/agg_plan/accs.csv");
    for(int i =0;i<(int)accs.size();++i)
    {
        accfile<<accs[i]->t<<","<<accs[i]->x<<","<<accs[i]->y<<","<<accs[i]->z<<endl;
    }
    accfile.close();

    cout<<"all write!"<<endl;
}

void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal)
{
    Eigen::Vector3i startgrid = pose2grid(startx,starty,startz);
    Eigen::Vector3i goalgrid = pose2grid(goalx,goaly,goalz);
    Node* startPos = new Node(startgrid(0),startgrid(1),startgrid(2));
    Node* goalPos = new Node(goalgrid(0),goalgrid(1),goalgrid(2));
    auto t1 = ros::Time::now();
    Astar astar(startPos,goalPos,pmap,sizex,sizey,sizez);
    path = astar.search();  //间距为0.1
    //剪切路径
    cutpath = astar.cutPath(path);
    //将栅格坐标转换为word坐标
    for(int i = 0;i<(int)cutpath.size();++i)
    {
        Eigen::Vector3d pose = grid2pose(cutpath[i]->x,cutpath[i]->y,cutpath[i]->z);
        Point* pt = new Point(pose(0),pose(1),pose(2));
        //shared_ptr<Point> pt = make_shared<Point>(pose(0),pose(1),pose(2));
        Path.push_back(pt);
    }
    //细分路径
    optPath = divide_path(Path);
    //计算节点长度
    init_time(Bspparam.poseVector,optPath);
    //调整控制点
    aggadjust_pts(optPath);
    //由控制点进行时间重调整
    adjust_time(optPath);
    //Bspline 由控制点计算位置
    cal_bspline(optPath,route,Bspparam.k,Bspparam.poseVector);
    cout<<"route:"<<route.size()<<endl;
    //由控制点计算速度和加速度
    cout<<"before: "<<vts.size()<<" "<<ats.size()<<endl;
    cal_vtats(optPath,vts,ats);
    cout<<"after: "<<vts.size()<<" "<<ats.size()<<endl;
    cout<<vts[0]->x<<" "<<vts[0]->y<<" "<<vts[0]->z<<endl;
    cout<<ats[0]->x<<" "<<ats[0]->y<<" "<<ats[0]->z<<endl;
    for(int i = 1;i<(int)Bspparam.poseVector.size()-1;++i)
    {
        Bspparam.velVector.push_back(Bspparam.poseVector[i]);
    }
    for(int i =2;i<(int)Bspparam.poseVector.size()-2;++i)
    {
        Bspparam.accVector.push_back(Bspparam.poseVector[i]);
    }
    cal_bspline(vts,vels,Bspparam.k-1,Bspparam.velVector);
    cal_bspline(ats,accs,Bspparam.k-2,Bspparam.accVector);
    write_route();
    auto t2 = ros::Time::now();
    cout<<"time used:"<<(t2-t1).toSec()<<"s"<<endl;
    cout<<"all ok!"<<endl;
}


void publish_path(const ros::TimerEvent& event)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp=ros::Time::now();
    path_msg.header.frame_id="map";
    for(int i=0;i<(int)cutpath.size();++i)
    {
        geometry_msgs::PoseStamped pose;
        Eigen::Vector3d pos = grid2pose(cutpath[i]->x,cutpath[i]->y,cutpath[i]->z);
        //cout<<path[i]->x<<"  "<<path[i]->y<<"  "<<path[i]->z<<endl;

        pose.pose.position.x=pos(0);
        pose.pose.position.y=pos(1);
        pose.pose.position.z=pos(2);

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        path_msg.poses.push_back(pose);
    }
    path_pub.publish(path_msg);
}

void publish_optimpath(const ros::TimerEvent& event)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp=ros::Time::now();
    path_msg.header.frame_id="map";
    for(int i=0;i<(int)optPath.size();++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x=optPath[i]->x;
        pose.pose.position.y=optPath[i]->y;
        pose.pose.position.z=optPath[i]->z;

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        path_msg.poses.push_back(pose);
    }
    optpath_pub.publish(path_msg);
}

int main(int argc, char *argv[])
{
    cout<<"hello!"<<endl;
    ros::init(argc,argv,"aggplan_node");
    ros::NodeHandle nh("~");

    //障碍物发布节点
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/env_pub",1);
    ros::Timer env_timer = nh.createTimer(ros::Duration(1),publish_env);

    //step1.环境创建
    //读取障碍物信息并发布
    init_param();
    init_env();
    read_env();
    //订阅目标点信息
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000,goalCallback);
    path_pub = nh.advertise<nav_msgs::Path>("/astar3d_path",10);
    ros::Timer path_timer = nh.createTimer(ros::Duration(1),publish_path);
    optpath_pub = nh.advertise<nav_msgs::Path>("/optim_astar3d_path",10);
    ros::Timer optimpath_timer = nh.createTimer(ros::Duration(1),publish_optimpath);
    ros::spin();
    return 0;
}
