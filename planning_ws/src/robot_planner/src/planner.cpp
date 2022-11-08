#include<iostream>
#include<planner.h>
#include<Astar.h>
using namespace std;

int sizex = 100;
int sizey = 100;
int** pmap;
map<int, int> near_map;

struct Point
{
    double x, y;
    Point(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    void set(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
};

vector<Point *> route_esdf;
vector<Point *> route_smooth;
vector<Node *> path;

ros::Publisher path_pub;
ros::Publisher route_smooth_pub;
ros::Publisher route_esdf_pub;
ros::Publisher obs_pub;

int  xy_to_idx(int x, int y)
{
    int idx = y*sizex + x;
    return idx;
}

void idx_to_xy(int idx, int& x, int& y)
{
    y = idx/sizex;
    x = idx - y*sizex; 
}

vector<Point *> node_to_point(vector<Node *> path)
{
    vector<Point *> rt;
    for (int i = 0; i < (int)path.size(); i++)
    {
        double idx = path[i]->x;
        double idy = path[i]->y;
        Point *pt = new Point(idx, idy);
        rt.push_back(pt);
    }
    return rt;
}

void initMap()
{
    pmap = (int **)malloc(sizex * sizeof(int *));
    for(int i=0; i<sizex;i++)
    {
        pmap[i] = (int *) malloc(sizey*sizeof(int));
    }
    for(int x=0;x<sizex;x++)
    {
        for(int y=0;y<sizey;y++)
        {
            pmap[x][y]=0;
        }
    }
    for(int x=15;x<40;x++)
    {
        for(int y=15;y<40;y++)
        {
            pmap[x][y]=1;
        }
    }
    for(int x=65;x<80;x++)
    {
        for(int y=65;y<80;y++)
        {
            pmap[x][y]=1;
        }
    }

    for(int x=15;x<40;x++)
    {
        for(int y=65;y<80;y++)
        {
            pmap[x][y]=1;
        }
    }

    for(int x=65;x<80;x++)
    {
        for(int y=15;y<40;y++)
        {
            pmap[x][y]=1;
        }
    }
}

vector<Point *> smooth_path(vector<Point *> path, double wd, double ws)
{
    vector<Point *> newpath;
    for (int i = 0; i < (int)path.size(); i++)
    {
        newpath.push_back(new Point(path[i]->x, path[i]->y));
    }

    for (int epoch = 0; epoch < 200; epoch++)
    {
        for (int i = 1; i < (int)path.size() - 1; i++)
        {
            newpath[i]->x = newpath[i]->x + wd * (path[i]->x - newpath[i]->x) + ws * (newpath[i - 1]->x + newpath[i + 1]->x - 2 * newpath[i]->x);
            newpath[i]->y = newpath[i]->y + wd * (path[i]->y - newpath[i]->y) + ws * (newpath[i - 1]->y + newpath[i + 1]->y - 2 * newpath[i]->y);
        }
    }
    return newpath;
}

bool isObstacle(int x, int y)
{
    if (x <0 || x >=sizex || y <0 || y >= sizey)
    {
        return true;
    }

    if (pmap[x][y]==1)
    {
        return true;
    }

    return false;
}

vector<Point*> obs_cir(int cx, int cy, int r)
{
    vector<Point*> obs_pt;
    double x = 0;
    double y = 0 + r;
    double d = 1.25-r; 
    
    obs_pt.push_back(new Point(cx+x,cy+y));
    if(isObstacle(cx+x,cy+y))
    return obs_pt;
    obs_pt.push_back(new Point(cx+x,cy-y));
        if(isObstacle(cx+x,cy-y))
    return obs_pt;
    obs_pt.push_back(new Point(cx-x,cy+y));
        if(isObstacle(cx-x,cy+y))
    return obs_pt;
    obs_pt.push_back(new Point(cx-x,cy-y));
        if(isObstacle(cx-x,cy-y))
    return obs_pt;
    obs_pt.push_back(new Point(cx+y,cy+x));
        if(isObstacle(cx+y,cy+x))
    return obs_pt;
    obs_pt.push_back(new Point(cx+y,cy-x));
            if(isObstacle(cx+y,cy-x))
    return obs_pt;
    obs_pt.push_back(new Point(cx-y,cy+x));
            if(isObstacle(cx-y,cy+x))
    return obs_pt;
    obs_pt.push_back(new Point(cx-y,cy-x));
            if(isObstacle(cx-y,cy-x))
    return obs_pt;
    while (x < y)
    {
        if (d <= 0){
            d = d + 2*x + 3;
            x = x + 1;
        }
        else
        {
            d = d + 2*(x-y) + 5;
            x = x+1;
            y = y-1;
        }
    obs_pt.push_back(new Point(cx+x,cy+y));
    if(isObstacle(cx+x,cy+y))
    return obs_pt;
    obs_pt.push_back(new Point(cx+x,cy-y));
        if(isObstacle(cx+x,cy-y))
    return obs_pt;
    obs_pt.push_back(new Point(cx-x,cy+y));
        if(isObstacle(cx-x,cy+y))
    return obs_pt;
    obs_pt.push_back(new Point(cx-x,cy-y));
        if(isObstacle(cx-x,cy-y))
    return obs_pt;
    obs_pt.push_back(new Point(cx+y,cy+x));
        if(isObstacle(cx+y,cy+x))
    return obs_pt;
    obs_pt.push_back(new Point(cx+y,cy-x));
            if(isObstacle(cx+y,cy-x))
    return obs_pt;
    obs_pt.push_back(new Point(cx-y,cy+x));
            if(isObstacle(cx-y,cy+x))
    return obs_pt;
    obs_pt.push_back(new Point(cx-y,cy-x));
            if(isObstacle(cx-y,cy-x))
    return obs_pt;
    }
    return obs_pt;
}


Point* near_obs(double x, double y, double rmin, double rmax)
{
    int idx = xy_to_idx(x,y);
    int idx_obs = near_map[idx];
    if (idx_obs!=0)
    {
        int xx,yy;
        idx_to_xy(idx_obs,xx,yy);
        Point* ob = new Point(xx,yy);
        return ob;
    }
    Point* ob = new Point(-100,-100);
    for(int r = rmin;r <=rmax;r++)
    {
        auto obs = obs_cir(x,y,r);
        for(int i = 0; i< (int)obs.size(); i++)
        {
            if (isObstacle(obs[i]->x,obs[i]->y))
            {
                int idx = xy_to_idx(int(x),int(y));
                near_map[idx]=xy_to_idx(obs[i]->x,obs[i]->y);
                ob = obs[i];
                return ob;
            }
        }
    }
}

double get_esdf(double x, double y,double nearx, double neary)
{
    double dist = sqrt((x-nearx)*(x-nearx) + (y-neary)*(y-neary));
    //double dist = (x-nearx)*(x-nearx) + (y-neary)*(y-neary);
    double esdf = 0;
    if (dist > 10)
    {
        esdf = 0;
    }
    else
    {
        esdf = (10-dist);
    }
    return esdf;
}

Point* get_grad(double x, double y)
{
    Point* grad = new Point(0,0);
    Point* near = near_obs(x,y,1,15);
    double nearx = near->x;
    double neary = near->y;
    grad->x = (get_esdf(x+0.01,y,nearx,neary) - get_esdf(x-0.01,y,nearx,neary))/0.02;
    grad->y = (get_esdf(x,y+0.01,nearx,neary) - get_esdf(x,y-0.01,nearx,neary))/0.02;
    return grad;
}

vector<Point *>  esdf_path(vector<Point *> path, double ws,double wo)
{
    vector<Point *> newpath;
    for (int i = 0; i < (int)path.size(); i++)
    {
        newpath.push_back(new Point(path[i]->x, path[i]->y));
    }
    for (int epoch = 0; epoch < 200; epoch++)
    {
        for (int i = 1; i < (int)path.size() - 1; i++)
        {
            auto grad = get_grad(newpath[i]->x,newpath[i]->y);
            double gradx = grad->x;
            double grady = grad->y;
            newpath[i]->x = newpath[i]->x + ws * (newpath[i - 1]->x + newpath[i + 1]->x - 2 * newpath[i]->x) - wo * gradx;
            newpath[i]->y = newpath[i]->y + ws * (newpath[i - 1]->y + newpath[i + 1]->y - 2 * newpath[i]->y) - wo * grady;
        }
    }
    return newpath;
}

void publish_path(const ros::TimerEvent &event)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "t265_odom_frame";

    for(int i=0; i<(int)path.size();i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = path[i]->x*0.1;
        pose_stamped.pose.position.y = path[i]->y*0.1;

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        path_msg.poses.push_back(pose_stamped);
    }
    path_pub.publish(path_msg);

    nav_msgs::Path route_esdf_msg;
    route_esdf_msg.header.stamp = ros::Time::now();
    route_esdf_msg.header.frame_id = "t265_odom_frame";

    for(int i=0; i<(int)route_esdf.size();i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = route_esdf[i]->x*0.1;
        pose_stamped.pose.position.y = route_esdf[i]->y*0.1;

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        route_esdf_msg.poses.push_back(pose_stamped);
    }
    route_esdf_pub.publish(route_esdf_msg);

    nav_msgs::Path route_smooth_msg;
    route_smooth_msg.header.stamp = ros::Time::now();
    route_smooth_msg.header.frame_id = "t265_odom_frame";
    for(int i=0; i<(int)route_smooth.size();i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = route_smooth[i]->x*0.1;
        pose_stamped.pose.position.y = route_smooth[i]->y*0.1;

        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;
        pose_stamped.pose.orientation.w = 1;

        route_smooth_msg.poses.push_back(pose_stamped);
    }
    route_smooth_pub.publish(route_smooth_msg);
}

void publish_obs(const ros::TimerEvent &event)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int x = 0; x < sizex; x++)
    {
        for(int y=0; y<sizey; y++)
        {
            //高度大于某个值也不发布
            if (pmap[x][y] == 0)
                continue;
            pt.x = x*0.1;
            pt.y = y*0.1;
            pt.z = 0;

            cloud.push_back(pt);
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "t265_odom_frame";

    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    obs_pub.publish(cloud_msg);
}

//计算轨迹长度
double cal_length(vector<Point *> path)
{
    double dist = 0;
    for(int i=0;i<(int)path.size()-1;i++)
    {
        double x1 = path[i]->x;
        double y1 = path[i]->y;
        double x2 = path[i+1]->x;
        double y2 = path[i+1]->y;
        dist = dist + pow(pow(x2-x1,2)+pow(y2-y1,2),0.5);
    }
    return dist;
}

//计算轨迹平均距离
double cal_dist_average(vector<Point *> path)
{
    double dist = 0;
    for(int i=0;i<(int)path.size();i++)
    {
        double x = path[i]->x;
        double y = path[i]->y;
        Point* near = near_obs(x,y,1,20);
        dist = dist + pow(pow(near->x-x,2)+pow(near->y-y,2),0.5);
    }
    return dist/path.size();
}

//计算轨迹最小距离
double cal_dist_min(vector<Point *> path)
{
    double dist_min = 1000;
    double dist = 1000;
    for(int i=0;i<(int)path.size();i++)
    {
        double x = path[i]->x;
        double y = path[i]->y;
        Point* near = near_obs(x,y,1,20);
        dist = pow(pow(near->x-x,2)+pow(near->y-y,2),0.5);
        if (isObstacle(x,y))
        {
            dist = 0;
        }
        if (dist < dist_min)
        {
            dist_min = dist;
        }
    }
    return dist_min;
}

//保存路径到csv
void save_path(vector<Point*> path,string name)
{
    ofstream file(string("/home/wuzp/temp_file/")+string(name));
    if (file)
    {
        for(int i=0;i<(int)path.size();i++)
        {
            file<<path[i]->x<<","<<path[i]->y<<"\n";
        }
    }
    file.close();
}

int main(int argc,char** argv)
{
    cout<<"planner"<<endl;
 
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");

    //step1 构建虚拟地图
    initMap();

    //step2 A*寻找路径点
    Node* startPos = new Node(8,8);
    Node* goalPos = new Node(90,90);
    Astar astar(startPos,goalPos,pmap,sizex,sizey);
    path = astar.search();
    auto p1 = node_to_point(path);
    double length = cal_length(p1);
    double dist = cal_dist_average(p1);
    double min_dist = cal_dist_min(p1);
    cout<<endl;
    cout<<"*******************"<<endl;
    cout<<"leng of A*: "<<length<<endl;
    cout<<"averge dist of A*: "<<dist<<endl;
    cout<<"min dist of A*: "<<min_dist<<endl;
    save_path(p1,"astar.csv");
    cout<<"*******************"<<endl;

    //step3 对路径点进行平滑
    ros::Time t1, t2;
    t1 = ros::Time::now();
    for (int i=0;i<100;i++)
    {
    route_esdf = esdf_path(p1,0.5,0.1);    
    }
    t2 = ros::Time::now();
    cout<<"esdf time durtion: "<<(t2 - t1).toSec()/100<<endl;
    length = cal_length(route_esdf);
    dist = cal_dist_average(route_esdf);
    min_dist = cal_dist_min(route_esdf);
    cout<<"leng of esdf: "<<length<<endl;
    cout<<"averge dist of esdf: "<<dist<<endl;
    cout<<"min dist of esdf: "<<min_dist<<endl;
    save_path(route_esdf,"esdf.csv");
    cout<<"*******************"<<endl;


    //step4 对路径进行光滑
    t1 = ros::Time::now();
    route_smooth = smooth_path(p1,0.1,0.5);
    t2 = ros::Time::now();
    cout<<"smooth time duration: "<<(t2-t1).toSec()<<endl;
    length = cal_length(route_smooth);
    dist = cal_dist_average(route_smooth);
    min_dist = cal_dist_min(route_smooth);
    cout<<"leng of smooth: "<<length<<endl;
    cout<<"averge dist of smooth: "<<dist<<endl;
    cout<<"min dist of  smooth: "<<min_dist<<endl;
    save_path(route_smooth,"smooth.csv");
    cout<<"*******************"<<endl;

    //step5 发布两条路径
    path_pub = nh.advertise<nav_msgs::Path>("/planning/astar",10);
    route_smooth_pub = nh.advertise<nav_msgs::Path>("/planning/smooth", 10);
    route_esdf_pub = nh.advertise<nav_msgs::Path>("/planning/esdf", 10);
    obs_pub = nh.advertise<sensor_msgs::PointCloud2>("/mymap/obs", 1, true);

    ros::Timer obs_timer = nh.createTimer(ros::Duration(0.5),publish_obs);
    ros::Timer path_timer = nh.createTimer(ros::Duration(0.5), publish_path);

    ros::spin();
}