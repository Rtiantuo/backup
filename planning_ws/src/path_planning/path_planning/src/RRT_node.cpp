#include<ros/ros.h>
#include<iostream>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/Marker.h>
#include<cstdlib>
#include<ctime>
#include"path_planning/RRT.h"
#include <Eigen/StdVector>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Eigen>

using namespace std;

int** pmap;
int sizex = 100;
int sizey = 100;

void init_map() 
{
    pmap=(int **) malloc(sizex*sizeof(int*));
    for(int i=0;i<sizex;++i)
    {
        pmap[i]=(int*)malloc(sizey*sizeof(int));
    }

    for(int x=0;x<sizex;++x)
    {
        for(int y=0;y<sizey;++y)
        {
            pmap[x][y]=0;
        }
    }

    int width=13;
    int centx=30;
    int centy=30;
    for(int x=centx-width;x<centx+width;++x)
    {
        for(int y=centy-width;y<centy+width;++y)
        {
            pmap[x][y]=1;
        }
    }

    centx=30;
    centy=70;
     for(int x=centx-width;x<centx+width;++x)
    {
        for(int y=centy-width;y<centy+width;++y)
        {
            pmap[x][y]=1;
        }
    }
}

void init_cloud(sensor_msgs::PointCloud2& cloud_msg)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int x=0;x<sizex;++x)
    {
        for(int y=0;y<sizey;++y)
        {
             if (pmap[x][y]==0)
            {
                continue;
            }
            pt.x=x*0.1;
            pt.y=y*0.1;
            pt.z=0;
            cloud.push_back(pt);
        }
    }
    cloud.width=cloud.points.size();
    cloud.height=1;
    cloud.is_dense=true;
    cloud.header.frame_id="map";
    pcl::toROSMsg(cloud,cloud_msg);
}

void init_markers(visualization_msgs::Marker& startPoint,
    visualization_msgs::Marker& goalPoint,
    visualization_msgs::Marker& randomPoint,
    visualization_msgs::Marker& RRTreeMarker,
    visualization_msgs::Marker& pathMarker)
{
    //init headers
    startPoint.header.frame_id = goalPoint.header.frame_id = randomPoint.header.frame_id = RRTreeMarker.header.frame_id = pathMarker.header.frame_id ="map";
    startPoint.header.stamp = goalPoint.header.stamp = randomPoint.header.stamp = RRTreeMarker.header.stamp =pathMarker.header.stamp= ros::Time::now();
    startPoint.ns = goalPoint.ns = randomPoint.ns = RRTreeMarker.ns = pathMarker.ns= "map";
    startPoint.action = goalPoint.action = randomPoint.action = RRTreeMarker.action = pathMarker.action = visualization_msgs::Marker::ADD;
    startPoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = RRTreeMarker.pose.orientation.w = pathMarker.pose.orientation.w= 1.0;

    //setting id for each marker
    startPoint.id = 0;
    goalPoint.id = 1;
    randomPoint.id = 2;
    RRTreeMarker.id = 3;
    pathMarker.id = 4;

    //define types 
    startPoint.type = goalPoint.type = randomPoint.type= visualization_msgs::Marker::SPHERE;
    RRTreeMarker.type = visualization_msgs::Marker::LINE_LIST; //两点两点组成的线段序列
    pathMarker.type = visualization_msgs::Marker::LINE_STRIP;

    //settign scale 
    startPoint.scale.x = goalPoint.scale.x = 0.2;
    startPoint.scale.y = goalPoint.scale.y = 0.2;
    startPoint.scale.z = goalPoint.scale.z =0.1;
    randomPoint.scale.x = randomPoint.scale.y = randomPoint.scale.z=0.1;
    RRTreeMarker.scale.x = 0.02;
    pathMarker.scale.x =0.06;

    //setting colors
    startPoint.color.r = 1.0f;
    goalPoint.color.g = 1.0f;
    randomPoint.color.b = 1.0f;

    RRTreeMarker.color.r = 0.8f;
    RRTreeMarker.color.g = 0.4f;
    pathMarker.color.g =1.0f;

    startPoint.color.a = goalPoint.color.a = randomPoint.color.a = RRTreeMarker.color.a = pathMarker.color.a = 1.0f;
}

void addBrach2Tree(visualization_msgs::Marker& RRTreeMarker,Eigen::Vector2d subPt,Eigen::Vector2d prePt)
{
    geometry_msgs::Point point;
    point.x = subPt(0)*0.1;
    point.y = subPt(1)*0.1;
    point.z = 0;
    RRTreeMarker.points.push_back(point);

    point.x = prePt(0)*0.1;
    point.y = prePt(1)*0.1;
    point.z = 0;
    RRTreeMarker.points.push_back(point);
}

void setPathMarker(visualization_msgs::Marker& pathMarker, vector<Node*>path)
{
    geometry_msgs::Point point;
    for(int i = 0;i<(int)path.size();++i)
    {
        point.x = path[i]->position(0)*0.1;
        point.y = path[i]->position(1)*0.1;
        point.z = 0;
        pathMarker.points.push_back(point);
    }
}


int main(int argc, char *argv[])
{
    cout<<"hello!"<<endl;
    ros::init(argc,argv,"RRT_node");
    ros::NodeHandle nh("~");
    init_map();
    //定义环境地图发布者对象
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/rrt_map",1);
    //定义起始点、目标点、RRTree发布者对象
    ros::Publisher RRT_publisher=nh.advertise<visualization_msgs::Marker>("/rrt_planner",1);

    sensor_msgs::PointCloud2 cloud_msg;
    init_cloud(cloud_msg);
    map_pub.publish(cloud_msg);

    visualization_msgs::Marker startPoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker RRTreeMarker;
    visualization_msgs::Marker pathMarker;

    init_markers(startPoint,goalPoint,randomPoint,RRTreeMarker,pathMarker);
    startPoint.pose.position.x = 0.2;
    startPoint.pose.position.y = 0.2;
    goalPoint.pose.position.x = 6.5;
    goalPoint.pose.position.y = 9.5;
    Eigen::Vector2d start(2.0,2.0);
    Eigen::Vector2d goal(65.0,95.0);
    int stepsize = 3;
    RRT rrt(start,goal,stepsize);

    RRT_publisher.publish(startPoint);
    RRT_publisher.publish(goalPoint);

    ros::spinOnce();
    ros::Duration(0.01).sleep();

    srand (time(NULL)); //随机数种子
    bool flag =true;
    //开始RRT搜索
    while(ros::ok() && flag)
    {
        //step1.生成随机点
        double x =rand() % (999 + 1) / (float)(999 + 1);
        double y =rand() % (999 + 1) / (float)(999 + 1);
        Eigen::Vector2d samPoint;
        if(x<0.5)
        {
            samPoint=goal;
        }
        else{
            samPoint<<x*100,y*100;
        }

        //step2.遍历树节点找到最近点
        int index = rrt.FindNearest(samPoint);
        //step3.生成子节点
        Eigen::Vector2d subPoint = rrt.newPoint(samPoint,rrt.Tree[index]->position);
        //step4.检查子节点
        bool feasible = rrt.checkPoint(pmap,subPoint,rrt.Tree[index]->position);  //可行点为true
        if (!feasible)  continue;
        //step5.将子节点加入Tree
        rrt.addTree(subPoint,index);
        //显示
        addBrach2Tree(RRTreeMarker,subPoint,rrt.Tree[index]->position);
        //step6.判断有无到达目标点附近  
        if((subPoint-goal).norm()<2)
        {
            rrt.addTree(goal,rrt.Tree.size()-1);
            cout<<"arrive goal!!"<<endl;
            rrt.retrivePath();
            setPathMarker(pathMarker,rrt.path);
            RRT_publisher.publish(pathMarker);
            flag = false;
        }

        map_pub.publish(cloud_msg);
        RRT_publisher.publish(startPoint);
        RRT_publisher.publish(goalPoint);
        RRT_publisher.publish(RRTreeMarker);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    return 0;
}
