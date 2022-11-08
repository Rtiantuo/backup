#include "build_map/mymap.h"

using namespace std;

struct Point
{
        double x,y;

        Point(double x, double y)
        {
            this->x = x;
            this->y = y;
        }

};

Eigen::Vector3d Pose;

double theat;

vector<Point*> route;
//    idx = near_pt(x,y,route);  寻找最近的点
int near_pt(double px, double py, vector<Point*> path)
{
    double d = 1000;
    int idx = 0;
    for (int i=0;i<(int) path.size()-1;i++)
    {
        double x = path[i]->x;
        double y = path[i]->y;
        double dist = pow(pow(px-x,2)+pow(py-y,2),0.5);
        if (dist < d)
        {
            d= dist;
            idx = i;
        }
    }
    cout<<"end"<<endl;
    return idx;
}

vector<Point*> gene_path(vector<Point*> path)
{
    vector<Point*>  newpath;
    for(int i=0;i<(int)path.size()-1;i++)
    {
        Point* pt1 = path[i];
        Point* pt2 = path[i+1];
        int num = pow((pow((pt2->x - pt1->x),2) + pow((pt2->y-pt1->y),2)),0.5)/0.02+0.05;
        double orx = pt1->x;
        double ory = pt1->y;
        double dx = (pt2->x - pt1->x)/double(num);
        double dy = (pt2->y - pt1->y)/double(num);
        for(int j = 0; j<num; j++)
        {
            double x = orx + dx*j;
            double y = ory + dy*j;
            Point* pt = new Point(x,y);
            newpath.push_back(pt);
        }
    }
    Point* pt = new Point(path[path.size()-1]->x,path[path.size()-1]->y);
    newpath.push_back(pt);
    return newpath;
}
//auto p1 = smooth_path(route,0.5,0.1); 在原路经的基础上，第一遍光滑路径
vector<Point*> smooth_path(vector<Point*> path, double wd, double ws)
{
    vector<Point*>  newpath;
    for(int i=0;i<(int)path.size();i++)
    {
        newpath.push_back(new Point(path[i]->x,path[i]->y));
    }

    for(int epoch = 0;epoch < 100; epoch++)
    {
        for(int i=1;i<(int)path.size()-1;i++)
        {
            newpath[i]->x = newpath[i]->x + wd * (path[i]->x - newpath[i]->x) + ws * (newpath[i-1]->x + newpath[i+1]->x-2*newpath[i]->x);
            newpath[i]->y = newpath[i]->y + wd * (path[i]->y - newpath[i]->y) + ws * (newpath[i-1]->y + newpath[i+1]->y-2*newpath[i]->y);
        }
    }
    return newpath;
}

void print_path(vector<Point*> path)
{
    for(int i=0;i<(int)path.size();i++)
    {
        cout<<"pt "<<i<<" :"<<path[i]->x<<" "<<path[i]->y<<endl;
    }
}

//手动生成一条路径
void myinit()
{
    for(int i=0;i<10;i++)
    {
        Point* pt = new Point(i*0.1,i*0.1);
        route.push_back(pt);
    }

    for(int j=0;j<10;j++)
    {
        Point* pt = new Point(1+j*0.1,1-j*0.1);
        route.push_back(pt);
    }

    for(int j=0;j<10;j++)
    {
        Point* pt = new Point(2,0-j*0.1);
        route.push_back(pt);
    }

    auto p1 = smooth_path(route,0.5,0.1);
    auto p2 = gene_path(p1);
    auto p3 = smooth_path(p2,0.1,0.5);
    route = p3;

    emit_cmd = false;
    theat = 0;
}



void odomCallback(const nav_msgs::OdometryConstPtr& odom)
{

    Pose(0) = odom->pose.pose.position.x;
    Pose(1) = odom->pose.pose.position.y;
    Pose(2) = odom->pose.pose.position.z;

    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation,q);

    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

    theat = yaw;

    double dist = pow(pow(Pose(0)-2,2) + pow(Pose(1)+1,2),0.5);
    if (dist < 0.2)
    {
        emit_cmd = false;
         geometry_msgs::Twist speed;
        double v = 0;
        double w = 0;
        speed.linear.x = v;
        speed.angular.z = w;
        cmd_pub_.publish(speed);
        return;
    }

    if (emit_cmd == false)
    {
        emit_cmd = true;
        t_begin = ros::Time::now();
    }
}

void cal_cmd(double & vel, double & w)
{
    vel = 0.08;

    //当前x,y
    double x = Pose(0);
    double y = Pose(1);

    //目标x,y
    int idx = 100;
    idx = near_pt(x,y,route); //找到距离当前最近的点
    cout<<"idx is: "<<idx<<endl;
    //最近点的坐标
    double rx = route[idx]->x;
    double ry = route[idx]->y;
    double rth = atan2(route[idx+1]->y-ry,route[idx+1]->x-rx);//最近点和下个点之间的角度

    //计算dist  最近点和当前点之间的距离
    double dist = pow(pow(rx-x,2) + pow(ry-y,2),0.5);

    //若dist小于某个设定之
    if (dist <= 0.2)
    {
        double dth = rth - theat;
          if(dth>3.14){
        dth=dth-6.28;
    }
    if(dth<-3.14){
        dth=dth+6.28;
    }
    w=0.8*dth;
    }
    else
    {
        double dth = atan2(ry-y,rx-x)-theat;
              if(dth>3.14){
        dth=dth-6.28;
    }
             if(dth<-3.14){
        dth=dth+6.28;
    }

        w = 0.8*dth;
    }

    //设定yuzhi
    if (w>0.5)  w=0.5;
    if (w<-0.5) w=-0.5;
}



//发布控制信息
void publish_cmd(const ros::TimerEvent& event)
{
    if(emit_cmd == false)
    {
        return;
    }
    //step3. 发布小车控制指令
    geometry_msgs::Twist speed;
    double v = 0;
    double w = 0;
    cal_cmd(v,w);
    speed.linear.x = v;
    speed.angular.z = w;
    cmd_pub_.publish(speed);
}

int main(int argc, char** argv)
{
    myinit();
    ros::init(argc,argv,"build_map");

    //print_path(p3);
	ros::NodeHandle nh("~");
    //订阅odom消息进行转化
    ros::Subscriber tt = nh.subscribe("/t265/odom/sample",1000, odomCallback);
    //控制消息的发布器
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //计时器4 发布小车控制
    ros::Timer cmd_timer_ = nh.createTimer(ros::Duration(0.1),publish_cmd);

    ros::spin();
}