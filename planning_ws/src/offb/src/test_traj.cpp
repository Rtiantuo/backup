//
// Created by lr on 2021/7/8.
//

#include "../include/offb/PolyTraj.h"
#include "vector"
#include "iostream"


int main()
{
    std::vector<double> a;
    a.push_back(2.0);
    a.push_back(-3.0);
    a.push_back(1.0);
    a.push_back(-2.0);
    PolyTraj traj_p(a, 3,1.);
    PolyTraj traj_v = traj_p.get_derivate();
    PolyTraj traj_a = traj_v.get_derivate();
    std::cout<<traj_v.get_value(1.5)<<std::endl;
    std::cout<<traj_a.get_value(1.5)<<std::endl;
    return 0;
}
