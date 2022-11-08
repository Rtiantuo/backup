//
// Created by lr on 2021/7/8.
//
#include "vector"
#ifndef OFFB_TRAJ_H
#define OFFB_TRAJ_H
#include "Eigen/Eigen"
#include "vector"
#include "iostream"
#include "nav_msgs/Odometry.h"
#define TRAJ_BEFORE 'a'
#define TRAJ_INCOMMING 'b'
#define TRAJ_FINISHED 'c'

class PolyTraj {
private:
    int order_; // 阶数
    std::vector<double> factors_; // 多项式系数
public:
    double t0_;
    double tau_; // 该轨迹时间段长度
    char traj_state(double t) const;

    void reinit(std::vector<double> fac, double tau, double t0);
    PolyTraj(std::vector<double> fac, double tau, double t0);
    PolyTraj();
    ~PolyTraj();
    PolyTraj get_derivate();
    double get_value(double t);
    void set_point(const double& x0, const double &t0);
};

std::vector<double> get_factors(double p0, double v0, double a0,
                               double pf, double vf, double af, double T);
double tau_estimate(const Eigen::Vector3d& p0,const Eigen::Vector3d& pf,double v_max, double a_max);

    #endif //OFFB_TRAJ_H
