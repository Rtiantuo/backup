#ifndef MYMAP_H
#define MYMAP_H

#include <iostream>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

#include<nav_msgs/Path.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Twist.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include<ctime>
#include<vector>
#include<queue>
#include<math.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include"occ_map/Astar.h"

using namespace std;
using namespace message_filters;



ros::Publisher cloud_pub_;
ros::Publisher map_pub_;
ros::Publisher path_pub_;
ros::Publisher cmd_pub_;
bool emit_cmd;
vector<Node*> path;

typedef struct gridindex_
{
    int x;
    int y;
    int z;
    void SetIndex(int x_, int y_, int z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }
} GridIndex;

typedef struct map_data
{
    Eigen::Vector3d Pose;

    Eigen::Matrix3d Rotation;

    double theat;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    bool has_cloud;

    int* p_map; 
} MapData;

typedef struct map_params
{
    double log_occ,log_free;
    double log_max,log_min;
    double resolution;
    double origin_x,origin_y,origin_z;
    int sizex,sizey,sizez;
    int offset_x,offset_y;
}MapParams;

MapData md;
MapParams mp;

void init()
{
    md.has_cloud = false;
    mp.sizex = 200;
    mp.sizey = 200;
    mp.sizez = 40;
    mp.resolution = 0.1;

    mp.origin_x = 0;
    mp.origin_y = 0;
    mp.origin_z = -1;

    mp.offset_x = mp.sizex/2.0;
    mp.offset_y = mp.sizey/2.0;

    md.p_map = new int[mp.sizex*mp.sizey*mp.sizez];
    for (int i=0;i<mp.sizex*mp.sizey*mp.sizez; i++)
    {
        md.p_map[i] = 50;
    }
}

//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(double x,double y,double z)
{
    GridIndex index;

    index.x = std::ceil((x - mp.origin_x) / mp.resolution) + mp.offset_x;
    index.y = std::ceil((y - mp.origin_y) / mp.resolution) + mp.offset_y;
    index.z = std::ceil((z  - mp.origin_z)/ mp.resolution);

    return index;
}

//栅格转世界坐标系
Eigen::Vector3d ConvertGridindex2World(int x, int y, int z)
{
    double px = (x - mp.offset_x)*mp.resolution+mp.origin_x;
    double py = (y - mp.offset_y)*mp.resolution+mp.origin_y;
    double pz = z*mp.resolution+mp.origin_z;
    return Eigen::Vector3d(px,py,pz);
}

//判断是否为有效格子
bool isValidGridIndex(GridIndex index)
{
    if (index.x >=0 && index.x < mp.sizex && index.y >= 0 && index.y < mp.sizey && index.z >= 0 && index.z < mp.sizez)
    {
        return true;
    }
    return false;
}

//将x,y,z索引转为线性一纬索引
int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.z *mp.sizex*mp.sizey + index.y*mp.sizex + index.x;
    return linear_index;
}

//一纬索引转坐标
Eigen::Vector3d indexToPos(int index)
{
    Eigen::Vector3d pos;
    int idz = std::ceil(index/(mp.sizex*mp.sizey));
    int idxy = index - idz*mp.sizex*mp.sizey;
    int idy = std::ceil(idxy/mp.sizex);
    int idx = idxy - idy*mp.sizex;

    double px = (idx - mp.offset_x)*mp.resolution+mp.origin_x;
    double py = (idy - mp.offset_y)*mp.resolution+mp.origin_y;
    double pz = idz*mp.resolution+mp.origin_z;
    return Eigen::Vector3d(px,py,pz);
}

//计算经过的网格
vector<GridIndex> raycast(int x1,int y1,int z1,int x2, int y2, int z2)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    int dx, dy, dz;
    int sx, sy, sz;

    dx  = abs(x2 - x1);
    dy  = abs(y2 - y1);
    dz  = abs(z2 - z1);

    if (x1 > x2)
    {
        sx = -1;
    }
    else
    {
        sx = +1;
    }

    if (y1 > y2)
    {
        sy = -1;
    }
    else
    {
        sy = +1;
    }

    if (z1 > z2)
    {
        sz = -1;
    }
    else
    {
        sz = +1;
    }

    int PX;
    int PY;
    int PZ;
    if (dx>=dy && dx >= dz)
    {
        int deltaX = abs(x2 - x1);
        int deltaY = abs(y2 - y1);
        int deltaZ = abs(z2 - z1);
        int errorY = 0;
        int errorZ = 0;
        int y = y1;
        int z = z1;
        for(int x = x1; x!=x2;x=x+sx)
        {
            PX = x;
            PY = y;
            PZ = z;

            errorY += deltaY;
            errorZ += deltaZ;

            if (2*errorY > deltaX)
            {
                y += sy;
                errorY -= deltaX;
            }

            if (2*errorZ > deltaX)
            {
                z += sz;
                errorZ -= deltaX;
            }

            if (PX == x2 && PY == y2 && PZ == z2) continue;

            tmpIndex.SetIndex(PX,PY,PZ);
            gridIndexVector.push_back(tmpIndex);
        }
    }

   else if (dy>=dx && dy >= dz)
    {
        int deltaX = abs(x2 - x1);
        int deltaY = abs(y2 - y1);
        int deltaZ = abs(z2 - z1);
        int errorX = 0;
        int errorZ = 0;
        int x = x1;
        int z = z1;
        for(int y = y1; y!=y2;y=y+sy)
        {
            PX = x; 
            PY = y;
            PZ = z;

            errorX += deltaX;
            errorZ += deltaZ;

            if (2*errorX > deltaY)
            {
                x += sx;
                errorX -= deltaY;
            }

            if (2*errorZ > deltaX)
            {
                z += sz;
                errorZ -= deltaY;
            }

            if (PX == x2 && PY == y2 && PZ == z2) continue;

            tmpIndex.SetIndex(PX,PY,PZ);
            gridIndexVector.push_back(tmpIndex);
        }
    }
    else
    {
        int deltaX = abs(x2 - x1);
        int deltaY = abs(y2 - y1);
        int deltaZ = abs(z2 - z1);
        int errorX = 0;
        int errorY = 0;
        int x = x1;
        int y = y1;
        for(int z = z1; z!=z2;z=z+sz)
        {
            PX = x; 
            PY = y;
            PZ = z;

            errorX += deltaX;
            errorY += deltaY;

            if (2*errorX > deltaZ)
            {
                x += sx;
                errorX -= deltaZ;
            }

            if (2*errorY > deltaZ)
            {
                y += sy;
                errorY -= deltaZ;
            }

            if (PX == x2 && PY == y2 && PZ == z2) continue;

            tmpIndex.SetIndex(PX,PY,PZ);
            gridIndexVector.push_back(tmpIndex);
        }
    }

    return gridIndexVector;
}

void update_occupancy();
void publish_map();
#endif