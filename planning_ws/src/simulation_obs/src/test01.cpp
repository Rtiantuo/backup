#include<ros/ros.h>
#include<iostream>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>

using namespace std;

ros::Publisher obs1_pub;
ros::Publisher circle_pub;
ros::Publisher cylinder_pub;
double resolution = 0.05;  // 1m20格
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

//矩形障碍物 
vector<Point*> rectangle_obs1;
void init_rectangle(double origin_x,double origin_y,double origin_z,double length,double width,double height,vector<Point*>& rectangle_obs)
{
    for(int i = 0; i<length; ++i)
    {
        for(int j = 0;j<width;++j)
        {
            for(int k = 0;k<height;++k)
            {
                double x =origin_x + i*resolution;
                double y =origin_y + j*resolution;
                double z = origin_z + k*resolution;
                //cout<<"xyz"<<x<<" "<<y<<" "<<z<<endl;
                Point* pt =new Point(x,y,z);
                rectangle_obs.push_back(pt);
            }
        }
    }
}

vector<Point*> circle_obs1;
void init_circle(double origin_x,double origin_y,double origin_z,double r,double thickness,vector<Point*>& circle_obs)
{
    //init_circle(4.0,4.0,1.5,  1.5,  0.5,circle_obs1);
    //3.5 2.5 0 
    double startx = origin_x  - thickness;
    double starty = origin_y - r;
    double startz = origin_z - r;
    for(double i = startx;i < (origin_x+thickness);i+=resolution)
    {
        for(double j = starty;j<(origin_y+r);j+=resolution)
        {
            for(double k =startz;k<(origin_z+r);k+=resolution)
            {
                //cout<<"xyz"<<i<<"  "<<j<<"  "<<k<<endl;
                double dis = (j - origin_y)*(j - origin_y) + (k - origin_z)*(k - origin_z);
               // cout<<dis<<endl;
                if(dis<r && dis > 0.5)
                {
                    Point* pt =new Point(i,j,k);
                    circle_obs.push_back(pt);
                }
            }
        }
    }
}

vector<Point*> cylinder_obs;
void init_cylinder(double origin_x,double origin_y,double origin_z,double r,double height,vector<Point*>& cylinder_obs)
{
    double start_x =origin_x - r;
    double start_y = origin_y -r;
    double start_z =origin_z;
    for(double i = start_x;i<(origin_x+r);i+=resolution)
    {
        for(double j = start_y;j<(origin_y+r);j+=resolution)
        {
            for(double k =start_z;k<height;k+=resolution)
            {
                double  dis = (i - origin_x)*(i - origin_x) + (j - origin_y)*(j - origin_y);
                if(dis < r )
                {
                    Point* pt =new Point(i,j,k);
                    cylinder_obs.push_back(pt);
                }
            }
        }
    }
}

void publish_map1(const ros::TimerEvent& event)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    for(int i = 0;i<(int)rectangle_obs1.size();++i)
    {
        pt.x = rectangle_obs1[i]->x;
        pt.y = rectangle_obs1[i]->y;
        pt.z = rectangle_obs1[i]->z;
        cloud1.push_back(pt);
    }
    cout<<"size:"<<cloud1.size()<<endl;
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud1_msg;
    pcl::toROSMsg(cloud1,cloud1_msg);
    obs1_pub.publish(cloud1_msg);
}

void publish_map2(const ros::TimerEvent& event)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    //cout<<"size:"<<circle_obs1.size()<<endl;
    for(int i = 0;i<(int)circle_obs1.size();++i)
    {
        pt.x = circle_obs1[i]->x;
        pt.y = circle_obs1[i]->y;
        pt.z = circle_obs1[i]->z;
        cloud2.push_back(pt);
    }
    cout<<"size:"<<cloud2.size()<<endl;
    cloud2.width = cloud2.points.size();
    cloud2.height = 1;
    cloud2.is_dense = true;
    cloud2.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_msg2;
    pcl::toROSMsg(cloud2,cloud_msg2);
    circle_pub.publish(cloud_msg2);
}

void publish_map3(const ros::TimerEvent& event)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud3;
    //cout<<"size:"<<circle_obs1.size()<<endl;
    for(int i = 0;i<(int)cylinder_obs.size();++i)
    {
        pt.x = cylinder_obs[i]->x;
        pt.y = cylinder_obs[i]->y;
        pt.z = cylinder_obs[i]->z;
        cloud3.push_back(pt);
    }
    cout<<"size:"<<cloud3.size()<<endl;
    cloud3.width = cloud3.points.size();
    cloud3.height = 1;
    cloud3.is_dense = true;
    cloud3.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_msg3;
    pcl::toROSMsg(cloud3,cloud_msg3);
    cylinder_pub.publish(cloud_msg3);
}


int main(int argc, char *argv[])
{

    ros::init(argc,argv,"obs_node");
    ros::NodeHandle nh("~");

    init_rectangle(1.0, 2.0, 0.0, 10, 80, 50, rectangle_obs1 );

    //rectangle_obs1发布者
    obs1_pub = nh.advertise<sensor_msgs::PointCloud2>("/obs1_map",1);
    ros::Timer obs1_timer=nh.createTimer(ros::Duration(0.5),publish_map1);

    init_circle(4.0,2.5,1.5,1.5,0.5,circle_obs1);
    circle_pub = nh.advertise<sensor_msgs::PointCloud2>("/obs2_map",1);
    ros::Timer obs2_timer = nh.createTimer(ros::Duration(0.5),publish_map2);

    init_cylinder(6.5,2.5,0,1.0,3,cylinder_obs);
    cylinder_pub = nh.advertise<sensor_msgs::PointCloud2>("/obs3_map",1);
    ros::Timer obs3_timer = nh.createTimer(ros::Duration(0.5),publish_map3);
    ros::spin();
    return 0;
}
