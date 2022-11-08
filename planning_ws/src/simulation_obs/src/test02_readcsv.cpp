#include<ros/ros.h>
#include<iostream>
#include<fstream>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<Eigen/Eigen>

using namespace std;
ros::Publisher map_pub;

struct Point{
    double x;
    double y;
    double z;
    Point(double x_,double y_,double z_)
    {
        this->x = x_;
        this->y = y_;
        this->z = z_;
    }
};
string file_name = "/home/tt/planning_ws/src/simulation_obs/src/env1.csv";

void publish_env(const ros::TimerEvent& event)
{
    cout<<"int the pub!"<<endl;
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //vector<Point*> points;
    //读取csv文件
    // ifstream file(file_name);
    // if(!file)
    // {
    //     cout<<"can not find env_file at: "<<file_name<<endl;
    //     return ;
    // }
    // while(!file.eof())
    // {
    //     double x,y,z;
    //      不能这样读取，因为中间有逗号
    //     file>>x>>y>>z;
    //     Point* point = new Point(x,y,z);
    //     points.push_back(point);
    // }

    ifstream file(file_name,ios::in);
    if (file.fail())
    {
        cout << "打开文件失败！" << endl;
        exit(1);
    }
    string line;
    while(getline(file,line))  //getline(inFile, line)表示按行读取CSV文件中的数据
    {
        string field;
        Eigen::Vector3d  point;
        istringstream sin(line);  //将整行字符串line读入到字符串流sin中
        int i = 0;
        if(line == " ") break;
        while(getline(sin,field,',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符 
        {
            point(i) = atof(field.c_str());
            ++i;
        }
        pt.x = point(0);
        pt.y = point(1);
        pt.z = point(2);
        cloud.push_back(pt);
    }
    cout<<"size: "<<cloud.size()<<endl;
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud,cloud_msg);
    map_pub.publish(cloud_msg);
}

int main(int argc, char *argv[])
{
    cout<<"hello!"<<endl;
    ros::init(argc,argv,"read_env");
    ros::NodeHandle nh("~");
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/env_pub",1);
    ros::Timer env_timer = nh.createTimer(ros::Duration(10),publish_env);
    ros::spin();
    return 0;
}
