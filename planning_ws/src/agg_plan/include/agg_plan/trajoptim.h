#ifndef TRAJOPTIM_H
#define TRAJOPTIM_H
#include<Eigen/Eigen>
#include<iostream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include"agg_plan/astar3d.h"
using namespace std;

// //点云kdtree
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
int k = 1;  //搜索点的数量
std::vector<int>pointIdxNKNsearch(k);   //保存搜索最近点的索引
 std::vector<float>pointNKNSquaredDistance(k);//保存最近距离

struct Point
{
    double x;
    double y;
    double z;
    double t;
    Point(double x_ ,double y_ ,double z_)
    {
        this->x = x_;
        this->y = y_;
        this->z = z_;
    }
    Point(double x_,double y_,double z_,double t_)
    {
        this->x = x_;
        this->y = y_;
        this->z = z_;
        this->t = t_;
    }
};

struct optim_param
{
    double w_smooth;
    double w_obs;
    double w_len;
    double thr_obs;
    double thr_len;
};
optim_param Optparam;

struct Bspline_param
{
    int k;
    double vmax;
    double amax ;
    vector<double> poseVector;
    vector<double> velVector;
    vector<double> accVector;
    Eigen::Vector3d startv;
    Eigen::Vector3d starta;
    Eigen::Vector3d goalv;
    Eigen::Vector3d goala;
};
Bspline_param Bspparam;

void init_param()
{
    Optparam.w_smooth = 0.2;
    Optparam.w_obs = -0.02;
    Optparam.w_len = 0.05;
    Optparam.thr_obs = 0.5;
    Optparam.thr_len = 0.1;
    Bspparam.k = 4;
    Bspparam.vmax = 2.0;
    Bspparam.amax = 10.0;
    Bspparam.startv <<1,0,0;
    Bspparam.starta <<0,0,0;
    Bspparam.goalv <<0,0,0;
    Bspparam.goala <<0,0,0;
}
void init_time(vector<double>& poseVector,vector<Point*>& path);
double cal_len(vector<Point*>& path);
void aggadjust_pts( vector<Point*>& path);
double cal_thr( vector<Point*>& path);
//计算梯度
Eigen::Vector3d cal_grad(const Point* path);
//计算光滑系数
Eigen::Vector3d cal_smooth(int index,vector<Point*>& path);
//计算长度损失
Eigen::Vector3d cal_length(int index,vector<Point*>& path);
//由控制点进行时间重调整
void  adjust_time(vector<Point*>path);
//Bspline由控制点计算位置
void cal_bspline(vector<Point*>path,vector<Point*>& route,int k,vector<double> posevector);
//根据pt计算每个点
Point cal_t(double t,int n,vector<Point*> path,int k ,vector<double> posevector);
//计算 B样条的基
double cal_Basic(int i,double t,int k,vector<double> posevector);
//由控制点计算速度和加速度
void cal_vtats(vector<Point*> path,vector<Point*>& vts,vector<Point*>& ats);

#endif