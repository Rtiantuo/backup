#ifndef _POLYCLOSEDFORMMINIMUMSNAP_H_
#define _POLYCLOSEDFORMMINIMUMSNAP_H_

#include <Eigen/Eigen>
#include <vector>
#include <iostream>

using namespace std;    
using namespace Eigen;

class PolyClosedForm_MinimumSnap{
    public:
        int Factorial(int x);
        VectorXd timeAllocation( MatrixXd Path);
        MatrixXd getQ(const int d_order,const MatrixXd &Path,const VectorXd time);
        MatrixXd getM(const int d_order,const MatrixXd &Path,const VectorXd time);
        MatrixXd getC(const int d_order,const MatrixXd &Path);
        MatrixXd PolyQPGeneration(const int d_order,const MatrixXd &Path,const MatrixXd &Vel,const MatrixXd &Acc,const VectorXd &Time);
};

#endif