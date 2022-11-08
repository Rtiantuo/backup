#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <vector>

//头文件
#include<iostream>
#include <fstream> 

using namespace std;
using namespace Eigen;



int main(int argc,char **argv)
{
    ros::init(argc,argv,"matlab");
    ros::NodeHandle n;

    ros::Publisher traj_pub = n.advertise<nav_msgs::Path>("/trajectory",1,true);
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";

    geometry_msgs::PoseStamped pt;
    pt.pose.orientation.x = 0;
    pt.pose.orientation.y = 0;
    pt.pose.orientation.z = 0;
    pt.pose.orientation.w = 1;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = "map";


    // read
    ifstream inFile("/home/tt/planning_ws/src/simulation_obs/src/path.csv",ios::in);
    if (inFile.fail())
    {
        cout << "打开文件失败！" << endl;
        exit(1);
    }

    string line;
    string field;
    while(getline(inFile,line))//getline(inFile, line)表示按行读取CSV文件中的数据
    {
        string field;
        istringstream sin(line); //将整行字符串line读入到字符串流sin中

        Vector3d pos;
        int i = 0;
        cout<<"line--"<<line<<endl;
        if (line==" ") break;
        while( getline(sin, field, ','))//将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
        {
            pos(i) = atof(field.c_str());
            i++;
        }
        
        pt.pose.position.x = pos(0);
        pt.pose.position.y= pos(1);
        pt.pose.position.z= pos(2);
        path.poses.push_back(pt);
        
    }
    inFile.close();

    while(ros::ok())
    {
        traj_pub.publish(path);
        ros::spin();
    }
   
}
