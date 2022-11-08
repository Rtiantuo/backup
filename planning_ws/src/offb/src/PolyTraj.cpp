//
// Created by lr on 2021/7/8.
//

#include "../include/offb/PolyTraj.h"

void PolyTraj::set_point(const double &x0, const double &t0) {
    std::vector<double> f0;
    f0.push_back(x0);
    reinit(f0,999999.,t0);
}

char PolyTraj::traj_state(double t) const {
    if ((t-t0_)<0) return TRAJ_BEFORE;
    else return t-t0_<tau_ ? TRAJ_INCOMMING : TRAJ_FINISHED;
//    std::cout<< t - t0_ << " -=- "<<tau_ <<std::endl;
//    return (t-t0_ < tau_)&&(t > t0_);
}

void PolyTraj::reinit(std::vector<double> fac, double tau, double t0) {
    factors_ = fac;
    order_ = fac.size()-1;
    tau_ = tau;
    t0_ = t0;
}

PolyTraj PolyTraj::get_derivate() {
    std::vector<double> dri_factors;
    if (order_<1)
    {
        dri_factors.push_back(0);
        return PolyTraj(dri_factors,tau_,t0_);
    }
//    for (int i = order_; i > 0 ; i--) {
//        dri_factors.push_back((order_ - i) * factors_[i]);
//    }
    for (int i = 0; i < order_; ++i) {
        dri_factors.push_back((order_ - i)*factors_[i]);
    }

//    std::reverse(dri_factors.begin(),dri_factors.end());
    return PolyTraj(dri_factors, tau_, t0_);
}

PolyTraj::PolyTraj(std::vector<double> fac, double tau, double t0) {
    factors_ = fac;
    order_ = fac.size()-1;
    tau_ = tau;
    t0_ = t0;
}

PolyTraj::PolyTraj() {
//    std::vector<double> f0;
//    f0.push_back(0.);
    factors_.clear();
    order_ = 0;
    tau_ = 0.0;
    t0_ = INFINITY;
}

double PolyTraj::get_value(double t) {
    if(order_==0)   return factors_[0];
    double result=0;
    for (int i = order_; i >= 0; i--) {
        result += factors_[i]* pow(t-t0_,order_-i);
    }
    return result;
}

PolyTraj::~PolyTraj() = default;

std::vector<double> get_factors(double p0,double v0, double a0,
                                double pf,double vf, double af, double T){
    /*求解多项式系数*/
    Eigen::Matrix<double,6,6> M;
    M<< 0,  0,  0,  0,  0,  1,
            0,  0,  0,  0,  1,  0,
            0,  0,  0,  2,  0,  0,
            1*pow(T, 5),    1*pow(T,4),     1*pow(T,3), 1*pow(T, 2),1*pow(T, 1),1*pow(T, 0),
            5*pow(T, 4),    4*pow(T,3),     3*pow(T,2), 2*pow(T, 1),1*pow(T, 0),0*pow(T, 0),
            20*pow(T, 3),   12*pow(T,2),    6*pow(T,1), 2*pow(T, 0),0*pow (T, 1),0*pow(T, 0);
    Eigen::Matrix<double,6,1> b;
    b<< p0,v0,a0,pf,vf,af;
    auto x = M.lu().solve(b);
    std::vector<double> result;
    for (int i = 0; i < x.size(); i++) {
        result.push_back(x[i]);
    }
    return result;
}

double tau_estimate(const Eigen::Vector3d& p0,const Eigen::Vector3d& pf,double v_max, double a_max){
    /*估计时间*/
    double euc_dis = (pf-p0).norm();
    double min_dis = 0.5*v_max*v_max/a_max;
    return euc_dis<min_dis? 2*sqrt(abs(euc_dis/a_max)) : 2 * v_max / a_max + (euc_dis - 2*min_dis) / v_max;
}
