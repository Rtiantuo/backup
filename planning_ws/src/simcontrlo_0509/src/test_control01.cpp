#include<ros/ros.h>
#include<iostream>
#include<fstream>
#include<geometry_msgs/Twist.h>
#include<visualization_msgs/Marker.h>
//uint8 TRIANGLE_LIST=11//三角形序列

using namespace std;
ros::Publisher pose_pub;
geometry_msgs::Twist this_vel;
visualization_msgs::Marker posMarker;
double cur_posex = 0.0;
double cur_posey = 0.0;
double this_velx = 0.0;
double this_vely = 0.0;
double dt = 0.1;
double cur_time = 0.0;

struct Point{
    double x;
    double y;
    double vx;
    double vy;
    double time;

    Point(double x_ ,double y_,double vx_,double vy_,double time_)
    {
        this->x = x_;
        this->y = y_;
        this->vx = vx_;
        this->vy = vy_;
        this->time = time_;
    }
};
vector<Point*> Points;


void velCallBack(const geometry_msgs::Twist::ConstPtr& vel)
{
    //cout<<"in the sub!"<<endl;
    this_vel = *vel;
    //cout<<"vel:"<<this_vel.linear.x<<"  "<<this_vel.linear.y<<endl;
}

//位置计算函数
void cal_pose(const ros::TimerEvent& event)
{
    cur_time += dt;
    this_velx +=  0.1*(this_vel.linear.x - this_velx);
    this_vely +=  0.1*(this_vel.linear.y - this_vely);
    cur_posex += this_velx*dt;
    cur_posey += this_vely*dt;
    //Point* pt = new Point(cur_posex,cur_posey,cur_time);
    Point* pt = new Point(cur_posex,cur_posey,this_velx,this_vely,cur_time);
    Points.push_back(pt);
    geometry_msgs::Point point;
    point.x = cur_posex;
    point.y = cur_posey;
    point.z = 0;
    posMarker.points.push_back(point);
    pose_pub.publish(posMarker);
}

void init_marker(visualization_msgs::Marker& posMarker)
{
    //init heads
    posMarker.header.frame_id = "map";
    posMarker.header.stamp = ros::Time::now();
    posMarker.ns  = "map";
    posMarker.action = visualization_msgs::Marker::ADD;
    posMarker.pose.orientation.w = 1.0;
    //setting id
    posMarker.id = 0;
    //define types 
    posMarker.type = visualization_msgs::Marker::SPHERE_LIST;  //三角形序列
    //setting scale
    posMarker.scale.x = 0.1;
    posMarker.scale.y = 0.1;
    posMarker.scale .z = 0.1;
    //setting color 
    posMarker.color.b = 1.0f;
    posMarker.color.a = 1.0f;


}

void write_data()
{
    cout<<"write!!"<<endl;
    ofstream outFile;
    outFile.open("/home/tt/planning_ws/src/simcontrlo_0509/src/pathData.csv",ios::out);
    for(int i=0;i<(int)Points.size();++i)
    {
        outFile<<Points[i]->time<<","<<Points[i]->x<<","<<Points[i]->y<<","<<Points[i]->vx<<","<<Points[i]->vy<<endl;
    }
    outFile.close();
    cout<<"all write!"<<endl;
}

int main(int argc, char *argv[])
{
    cout<<"hello!"<<endl;
    ros::init(argc,argv,"contrlo_node");
    ros::NodeHandle nh("~");
    init_marker(posMarker);
    //订阅速度
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("/turtle1/cmd_vel",1,velCallBack);
    //速度积分定时器
    ros::Timer  pose_timer = nh.createTimer(ros::Duration(0.1),cal_pose);
    //发布实时位置
    pose_pub = nh.advertise<visualization_msgs::Marker>("/pose_marker",1);
    ros::spin();

    cout<<"int the end write!"<<endl;
    write_data();
    return 0;
}


