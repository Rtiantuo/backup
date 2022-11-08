#include "PolyClosedForm_MinimumSnap.h"

int PolyClosedForm_MinimumSnap::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

VectorXd PolyClosedForm_MinimumSnap::timeAllocation( MatrixXd Path)
{ 
     double _Vel = 1; //  max v
     double _Acc = 1; //  max a
    //  VectorXd time(Path.rows() - 1);
    
//  //    time of  acc
//      double t_scope = _Vel/_Acc;
//  //    dist of acc(contain acc+ and acc-)
//      double distance_acc = 0.5*_Acc*t_scope*t_scope*2;

//      for(int i = 0;i<Path.rows()-1;i++)
//      {
//          Vector3d delta = Path.row(i) - Path.row(i+1);
//          double d = sqrt(delta.dot(delta));

//          if(d<=distance_acc){
//              time(i) = 2*d/_Acc;//sqrt(d/_Acc);
//          }
//          else{
//              time(i) = (d-distance_acc)/_Vel+t_scope;//2*t_scope + (d-distance_acc)/_Vel;
//          }
//      }
//      return time;

//    // 梯形时间分配
//    double _Vel = 3; //  max v
//    double _Acc = 3; //  max a
   VectorXd time(Path.rows() - 1);
   VectorXd dis(Path.rows() - 1);
//
//    double t_scope = _Vel/_Acc;
//    double distance_acc = 0.5*_Acc*t_scope;
//    bool flag1 = true; // 还可以继续加速
//    bool flag2 = true;
//
//    double d1 = 0;//走过的距离
//    double d2 = 0;
//
//    double time1 = 0;
//    double time2 = 0;
//
   for(int i = 0;i<Path.rows()-1;i++)
   {
       Vector3d delta = Path.row(i) - Path.row(i+1);
       dis(i) = sqrt(delta.dot(delta));
       time(i) = sqrt(dis(i));
   }
//
//    for (int i = 0,j = Path.rows()-2 ; i <= j;i++,j--)
//    {
//        if (flag1){
//            if ((dis(i)+d1)<distance_acc){
//                time(i) = sqrt(2* (dis(i) + d1)/_Acc) - time1 ;
//            }
//            else{
//                time(i) = t_scope + (dis(i) + d1 - distance_acc)/_Vel;//加速段+匀速段
//                flag1 = false;
//            }
//        }
//        else{
//            time(i) = dis(i)/_Vel;
//        }
//
//        d1 += dis(i);
//        time1 += time(i);
//
//        if (i!=j){
//            if(flag2){
//                if(dis(j)+d2<distance_acc){
//                    time(j) = sqrt(2* (dis(j) + d2)/_Acc) - time2 ;
//                }
//                else{
//                    time(j) = t_scope + (dis(j) + d2 - distance_acc)/_Vel;//加速段+匀速段
//                    flag2 = false;
//                }
//            }
//            else{
//                time(j) = dis(j)/_Vel;
//            }
//        }
//
//        d2 += dis(j);
//        time2 += time(j);
//
//    }
//    cout<<"dis"<<dis<<endl;
   return time;

}

MatrixXd PolyClosedForm_MinimumSnap::getQ(const int d_order,const MatrixXd &Path,const VectorXd time)
{
    //  seg order time
    int n_seg = Path.rows()-1;
    int n_order = 2*d_order-1;
    MatrixXd Q = MatrixXd::Zero((n_order+1)*n_seg,(n_order+1)*n_seg);
    for (int k = 0;k<n_seg;k++){
//        MatrixXd Q_k = MatrixXd::Zero(n_order+1,n_order+1);
        for (int i = 4;i<n_order+1;i++){
            for (int j=4;j<n_order+1;j++){
                double value = Factorial(i)/Factorial(i-4)*Factorial(j)/Factorial(j-4)/(i+j-7)*pow(time(k),i+j-7);
                Q(k*(n_order+1)+i,k*(n_order+1)+j) = value;
            }
        }
    }
    return Q;
}

MatrixXd PolyClosedForm_MinimumSnap::getM(const int d_order,const MatrixXd &Path,const VectorXd time)
{
    int n_seg = Path.rows()-1;
    int n_order = 2*d_order-1;
    MatrixXd M = MatrixXd::Zero((n_order+1)*n_seg,(n_order+1)*n_seg);
    for (int n = 0;n<n_seg;n++){

        MatrixXd M_k = MatrixXd::Zero(2*d_order,n_order+1);
        double T1 = 0;
        double T2 = time(n);
        for (int k = 0;k<=3;k++){
//            4个k -(p v a j)
            for (int i = k;i<=n_order;i++){
                M_k(k,i) = Factorial(i)/Factorial(i-k)*pow(T1,i-k);
                M_k(k+4,i) = Factorial(i)/Factorial(i-k)*pow(T2,i-k);
            }
        }
        M.block(n*(n_order+1),n*(n_order+1),2*d_order,n_order+1) = M_k;
    }
    return M;
}

MatrixXd PolyClosedForm_MinimumSnap::getC(const int d_order,const MatrixXd &Path)
{
    int n_seg = Path.rows()-1;
    int n_order = 2*d_order-1;

    int d_size = 2*d_order*n_seg;
    int dF_size = 2*d_order+n_seg-1;
    int dP_size = 3*(n_seg-1);
    MatrixXd C = MatrixXd::Zero(d_size,dF_size+dP_size);
//    dF--fixed dP--不
//   start and goal
//    p0 v0 a0 j0
    for (int i = 0;i<4;i++){
        C(i,i) = 1;
    }
//    pk vk ak jk
    for (int i = 0;i<4;i++){
        C(d_size-i-1,dF_size-1-i) = 1;
    }
    for (int k=1;k<n_seg;k++){
        int idx = 4+8*(k-1); //k-th seg p0

//        col:p
        int df_pk = 4+k;
        C(idx,df_pk) = 1;
        C(idx+4,df_pk)=1;

//        col: v a j
        int dp_vk = dF_size+3*(k-1);
        C(idx+1,dp_vk) = 1;
        C(idx+5,dp_vk) = 1;
        C(idx+2,dp_vk+1) = 1;
        C(idx+6,dp_vk+1) = 1;
        C(idx+7,dp_vk+2) = 1;
        C(idx+7,dp_vk+2) = 1;
    }
    return C;
}

MatrixXd PolyClosedForm_MinimumSnap::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
    

    MatrixXd Q = getQ(d_order,Path,Time);
//    cout<<"Q"<<Q<<endl;

    MatrixXd M = getM(d_order,Path,Time);
//    cout<<"M"<<M<<endl;

    MatrixXd C = getC(d_order,Path);
//    cout<<"C"<<C<<endl;

    MatrixXd R = C.transpose()*(M.inverse()).transpose()*Q*M.inverse()*C;
//    cout<<"R"<<R<<endl;
    const int n_seg = Path.rows()-1;

    MatrixXd R_pp = R.block(7+n_seg,7+n_seg,3*(n_seg-1),3*(n_seg-1));
    MatrixXd R_fp = R.block(0,7+n_seg,7+n_seg,3*(n_seg-1));

    MatrixXd jerk = MatrixXd::Zero(2,3);
    for (int i = 0;i<3;i++)
    {
        MatrixXd start_cond = MatrixXd(4,1);
        MatrixXd end_cond = MatrixXd(4,1);
        start_cond << Path(0,i),Vel(0,i),Acc(0,i),0;
        end_cond << Path(n_seg,i),Vel(1,i),Acc(1,i),0;
        MatrixXd dF = MatrixXd::Zero(7+n_seg,1);
        dF.block(0,0,4,1) = start_cond;
        dF.block(7+n_seg-4,0,4,1) = end_cond;
        dF.block(4,0,m-1,1) = Path.block(1,i,m-1,1);

        MatrixXd dP = -R_pp.inverse()*R_fp.transpose()*dF;
        MatrixXd d = MatrixXd(dF.rows()+dP.rows(),1);
        d<<dF,dP;

        MatrixXd poly_coef = M.inverse()*C*d;
//        cout<<"poly_coef"<<poly_coef<<endl;

        for (int j = 0;j<m;j++)
        {
            PolyCoeff.block(j,p_num1d*i,1,p_num1d) = poly_coef.transpose().block(0,j*p_num1d,1,p_num1d);
        }
    }


    return PolyCoeff;
}
