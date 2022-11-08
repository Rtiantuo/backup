#ifndef AGGPLAN_H
#define AGGPLAN_H
#include<ros/ros.h>
#include<vector>
#include<Eigen/Eigen>
#include<pcl/point_cloud.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<fstream>
#include"agg_plan/astar3d.h"
#include"agg_plan/trajoptim.h"
using namespace std;

inline double minx = 0.0;
inline double maxx =10.0;
inline double miny = -2.5;
inline double maxy = 2.5;
inline double minz = 0.0;
inline double maxz = 3;
inline double res = 0.1;
inline int sizex;
inline int sizey;
inline int sizez;
inline double startx = 1.0;
inline double starty = 2.0;
inline double startz = 1.0;
inline double goalx = 9.0;
inline double goaly = -1.5;
inline double goalz = 1.0;
inline vector<Node*> path;
inline int* pmap;
inline ros::Publisher map_pub;
inline ros::Publisher path_pub;
ros::Publisher optpath_pub;
inline vector<Point*> env_data;
vector<Point*> Path;
vector<Point*> optPath;
vector<Node*> cutpath;
vector<Point*> route;
vector<Point*> vts;
vector<Point*> vels;
vector<Point*> ats;
vector<Point*> accs;

inline void init_env()
{
    sizex = floor((maxx - minx)/res + 0.001);
    sizey = floor((maxy - miny)/res + 0.001);
    sizez = floor((maxz - minz)/res + 0.001);
    pmap = new int[sizex*sizey*sizez];
    for(int i = 0;i<sizex*sizey*sizez;++i)
    {
        pmap[i] = 0;  //空闲位置设为0
    }
}

//word坐标转换成栅格坐标
inline Eigen::Vector3i  pose2grid(double px,double py,double pz)
{
    int x = floor((px - minx)/res + 0.001);
    int y = floor((py - miny)/res + 0.001);
    int z = floor((pz - minz)/res + 0.001);
    return Eigen::Vector3i(x,y,z);
}

inline Eigen::Vector3d grid2pose(int x,int y,int z)
{
    double px = x*res + minx;
    double py = y*res + miny;
    double pz = z*res + minz;
    return Eigen::Vector3d(px,py,pz);
}

//细分路径
vector<Point*> divide_path(vector<Point*>& path)
{
    vector<Point*> new_path;
    Point* pt = new Point(path[0]->x,path[0]->y,path[0]->z);
    new_path.push_back(pt);
    for(int i=1;i<(int)path.size();++i)
    {
        double len = sqrt((path[i]->x - path[i-1]->x)*(path[i]->x - path[i-1]->x) + (path[i]->y - path[i-1]->y)*(path[i]->y - path[i-1]->y) + (path[i]->z - path[i-1]->z)*(path[i]->z - path[i-1]->z));
        double num = max(floor(len/0.1 +0.5),1.0);
        for(int k = 1;k<num+1;++k)
        {
            double px = path[i-1]->x + k/num*(path[i]->x - path[i-1]->x);
            double py = path[i-1]->y + k/num*(path[i]->y - path[i-1]->y);
            double pz = path[i-1]->z + k/num*(path[i]->z - path[i-1]->z);
            Point* pt = new Point(px,py,pz);
            new_path.push_back(pt);
        }
    }
    return new_path;
}

#endif