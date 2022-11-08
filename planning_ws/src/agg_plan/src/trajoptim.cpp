#include"agg_plan/trajoptim.h"

//计算节点
void init_time(vector<double>& poseVector,vector<Point*>& path)
{
    double len = cal_len(path);
    cout<<"path len: "<<len<<endl;
    int num = (int)path.size() -1 +Bspparam.k+2;
    double t = len/Bspparam.vmax;
    for(int i = 0;i< Bspparam.k;++i)
    {
        poseVector.push_back(0);
    }
    double interval = t/(num - 2*Bspparam.k - 1);
    for(int i = 0;i<(num - 2*Bspparam.k);++i)
    {
        poseVector.push_back(i*interval);
    }
    for(int i = 0;i<Bspparam.k;++i)
    {
        poseVector.push_back(t);
    }
}

double cal_len(vector<Point*>& path)
{
    double len = 0;
    for(int i = 0;i<(int)path.size()-1;++i)
    {
        len +=sqrt((path[i]->x - path[i+1]->x)*(path[i]->x - path[i+1]->x) + (path[i]->y - path[i+1]->y)*(path[i]->y - path[i+1]->y) + (path[i]->z - path[i+1]->z)*(path[i]->z - path[i+1]->z));
    }
    return len;
}

void aggadjust_pts(vector<Point*>& path)
{
    cout<<"start aggadjust!!"<<endl;
    vector<Point*> raw_path = path;
    //初始速度确定,第二个点确定
    double p1x = Bspparam.startv(0)*(Bspparam.poseVector[Bspparam.k+1] - Bspparam.poseVector[1])/Bspparam.k + path[0]->x;
    double p1y = Bspparam.startv(1)*(Bspparam.poseVector[Bspparam.k+1] - Bspparam.poseVector[1])/Bspparam.k + path[0]->y;
    double p1z = Bspparam.startv(2)*(Bspparam.poseVector[Bspparam.k+1] - Bspparam.poseVector[1])/Bspparam.k + path[0]->z;
    raw_path[1]->x = p1x;
    raw_path[1]->y = p1y;
    raw_path[1]->z = p1z;
    //初始加速度确定，第二个点速度确定，所以第三个点位置确定
    //计算速度v1
    double v1x = Bspparam.starta(0)*(Bspparam.poseVector[Bspparam.k+1] - Bspparam.poseVector[2])/(Bspparam.k - 1) +Bspparam.startv(0);
    double v1y = Bspparam.starta(1)*(Bspparam.poseVector[Bspparam.k+1] - Bspparam.poseVector[2])/(Bspparam.k - 1) +Bspparam.startv(1);
    double v1z = Bspparam.starta(2)*(Bspparam.poseVector[Bspparam.k+1] - Bspparam.poseVector[2])/(Bspparam.k - 1) +Bspparam.startv(2);
    //由v1计算p2
    double p2x = v1x/Bspparam.k *(Bspparam.poseVector[Bspparam.k+2] - Bspparam.poseVector[2]) + p1x;
    double p2y = v1y/Bspparam.k *(Bspparam.poseVector[Bspparam.k+2] - Bspparam.poseVector[2]) + p1y;
    double p2z = v1z/Bspparam.k *(Bspparam.poseVector[Bspparam.k+2] - Bspparam.poseVector[2]) + p1z;
    raw_path[2]->x = p2x;
    raw_path[2]->y = p2y;
    raw_path[2]->z = p2z;
    //终点速度确定，倒数第二个点位置确定
    int n = path.size();
    raw_path[n-2]->x = path[n-1]->x - Bspparam.goalv(0)*(Bspparam.poseVector[n-2+Bspparam.k+1]- Bspparam.poseVector[n-2+1])/Bspparam.k;
    raw_path[n-2]->y = path[n-1]->y - Bspparam.goalv(1)*(Bspparam.poseVector[n-2+Bspparam.k+1]- Bspparam.poseVector[n-2+1])/Bspparam.k;
    raw_path[n-2]->z = path[n-1]->z - Bspparam.goalv(2)*(Bspparam.poseVector[n-2+Bspparam.k+1]- Bspparam.poseVector[n-2+1])/Bspparam.k;
    //终点加速度确定，倒数第二个点速度确定，倒数第三个点位置确定
    //速度
    double  vend1x  =Bspparam.goalv(0) - Bspparam.goala(0)*(Bspparam.poseVector[n-3+Bspparam.k +1] - Bspparam.poseVector[n-3+2])/(Bspparam.k- 1);
    double  vend1y  =Bspparam.goalv(1) - Bspparam.goala(1)*(Bspparam.poseVector[n-3+Bspparam.k +1] - Bspparam.poseVector[n-3+2])/(Bspparam.k- 1);
    double  vend1z  =Bspparam.goalv(2) - Bspparam.goala(2)*(Bspparam.poseVector[n-3+Bspparam.k +1] - Bspparam.poseVector[n-3+2])/(Bspparam.k- 1);
    //由速度计算位置
    raw_path[n-3]->x = path[n-2]->x - vend1x*(Bspparam.poseVector[n-3+Bspparam.k+1]-Bspparam.poseVector[n-3+1])/Bspparam.k;
    raw_path[n-3]->y = path[n-2]->y - vend1y*(Bspparam.poseVector[n-3+Bspparam.k+1]-Bspparam.poseVector[n-3+1])/Bspparam.k;
    raw_path[n-3]->z = path[n-2]->z - vend1z*(Bspparam.poseVector[n-3+Bspparam.k+1]-Bspparam.poseVector[n-3+1])/Bspparam.k;

    for(int epoch = 0;epoch <100;++epoch)
    {
        if(epoch >50)
        {
            Optparam.thr_obs = cal_thr(raw_path)*1.05;
        }
        for(int i = 3;i<n-3;++i)
        {
            //计算梯度
            Eigen::Vector3d grad = cal_grad(raw_path[i]);
            //计算光滑系数
            Eigen::Vector3d smooth = cal_smooth(i,raw_path);
            //计算长度损失
            Eigen::Vector3d leng = cal_length(i,raw_path);
            raw_path[i]->x += Optparam.w_smooth*smooth(0) + Optparam.w_obs*grad(0) + Optparam.w_len*leng(0);
            raw_path[i]->y += Optparam.w_smooth*smooth(1) + Optparam.w_obs*grad(1) + Optparam.w_len*leng(1);
            raw_path[i]->z += Optparam.w_smooth*smooth(2) + Optparam.w_obs*grad(2) + Optparam.w_len*leng(2);
        }
    }
    cout<<"aggadjust OK!"<<endl;
}

//动态计算安全阈值
double cal_thr(vector<Point*>& path)
{
    double thrmin = 0.01;
    for(int i = 1;i<(int)path.size()-1;++i)
    {
        Eigen::Vector3d p1(path[i-1]->x,path[i-1]->y,path[i-1]->z);
        Eigen::Vector3d p2(path[i]->x,path[i]->y,path[i]->z);
        Eigen::Vector3d p3(path[i+1]->x,path[i+1]->y,path[i+1]->z);
        double smooth = (p3 + p1 - 2*p2).norm();
        double lmax = max((p3-p2).norm(),(p2-p1).norm());
        double thr = Bspparam.k /8*smooth + 0.5*lmax;
        if(thr > thrmin)
        {
            thrmin = thr;
        }
    }
    return thrmin;
}

//计算梯度
Eigen::Vector3d cal_grad(const Point* path)
{
    //查找最近点
    Eigen::Vector3d grad;
    vector<int> pointIdxNKNsearch(1);
    vector<float>pointNKNSquaredDistance(1);
    pcl::PointXYZ searchPoint;
    searchPoint.x = path->x;
    searchPoint.y = path->y;
    searchPoint.z = path->z;
    kdTree.nearestKSearch(searchPoint,1,pointIdxNKNsearch,pointNKNSquaredDistance);
    double dist = sqrt(pointNKNSquaredDistance[0]);
    if (dist > Optparam.thr_obs)
    {
        grad<<0,0,0;
        return grad;
    }
    double distx = path->x - cloudPtr->points[pointIdxNKNsearch[0]].x;
    double disty = path->y - cloudPtr->points[pointIdxNKNsearch[0]].y;
    double distz = path->z - cloudPtr->points[pointIdxNKNsearch[0]].z;
    double gradx = (dist - Optparam.thr_obs)/dist*2*distx;
    double grady = (dist - Optparam.thr_obs)/dist*2*disty;
    double gradz = (dist - Optparam.thr_obs)/dist*2*distz;
    return Eigen::Vector3d (gradx,grady,gradz);
}

//计算光滑损失
Eigen::Vector3d cal_smooth(int index,vector<Point*>& path)
{
    if(index > 1 && index <path.size()-2)
    {
        double smoothx = 2*(path[index-1]->x+path[index+1]->x-2*path[index]->x) - (path[index-2]->x+path[index]->x-2*path[index-1]->x)-(path[index]->x+path[index+2]->x-2*path[index+1]->x);
        double smoothy = 2*(path[index-1]->y+path[index+1]->y-2*path[index]->y) - (path[index-2]->y+path[index]->y-2*path[index-1]->y)-(path[index]->y+path[index+2]->y-2*path[index+1]->y);
        double smoothz = 2*(path[index-1]->z+path[index+1]->z-2*path[index]->z) - (path[index-2]->z+path[index]->z-2*path[index-1]->z)-(path[index]->z+path[index+2]->z-2*path[index+1]->z);
        return Eigen::Vector3d (smoothx,smoothy,smoothz);
    }
    else
    {
        double smoothx = 2*(path[index-1]->x+path[index+1]->x-2*path[index]->x);
        double smoothy = 2*(path[index-1]->y+path[index+1]->y-2*path[index]->y);
        double smoothz = 2*(path[index-1]->z+path[index+1]->z-2*path[index]->z);
        return Eigen::Vector3d (smoothx,smoothy,smoothz);
    }
}

//计算长度损失
Eigen::Vector3d cal_length(int index,vector<Point*>& path)
{   //当前点
    Eigen::Vector3d pt1(path[index]->x,path[index]->y,path[index]->z);
    //当前点的前一个点
    Eigen::Vector3d pt0(path[index-1]->x,path[index-1]->y,path[index]->z);
    //后一个点
    Eigen::Vector3d pt2(path[index+1]->x,path[index+1]->y,path[index+1]->z);
    double len1 = (pt1 - pt0).norm();
    double len2 = (pt2 - pt1).norm();
    Eigen::Vector3d length;
    if(len1 >= Optparam.thr_len)
    {
        length = 2*(len1 - Optparam.thr_len)/len1*2*(pt0 - pt1);
    }
    if(len2 >= Optparam.thr_len)
    {
        length += 2*(len2 - Optparam.thr_len)/len2*2*(pt2 - pt1);
    }
    return length;
}

//由控制点进行时间重调整
void  adjust_time(vector<Point*>path)
{
    for(int epoch = 0;epoch<100;++epoch)
    {
        for(int k = 2;k<(int)path.size()-3;++k)
        {
            double dt = Bspparam.poseVector[k+Bspparam.k +1]-Bspparam.poseVector[k+1];
            double dt1 = fabs(Bspparam.k*(path[k+1]->x - path[k]->x)/Bspparam.vmax);
            double dt2 = fabs(Bspparam.k*(path[k+1]->y - path[k]->y)/Bspparam.vmax);
            double dt3 = fabs(Bspparam.k*(path[k+1]->z - path[k]->z)/Bspparam.vmax);
            double mindt = max(dt1,max(dt2,dt3));
            if (dt < mindt)
            {
                for(int j = k +Bspparam.k+1 ;j < Bspparam.poseVector.size();++j)
                {
                    Bspparam.poseVector[j] += mindt - dt;
                }
            }
        }
        vector<Point*>  vts;
        for(int i = 0;i<(int)path.size()-1;++i)
        {
            double vx = Bspparam.k * (path[i+1]->x - path[i]->x )/(Bspparam.poseVector[i+Bspparam.k +1]-Bspparam.poseVector[i+1]);
            double vy = Bspparam.k * (path[i+1]->y - path[i]->y )/(Bspparam.poseVector[i+Bspparam.k +1]-Bspparam.poseVector[i+1]);
            double vz = Bspparam.k * (path[i+1]->z - path[i]->z )/(Bspparam.poseVector[i+Bspparam.k +1]-Bspparam.poseVector[i+1]);
            Point* vt = new Point(vx,vy,vz);
            vts.push_back(vt);
        } 

        for(int i = 1;i<(int)vts.size()-2;++i)
        {
            double dt = Bspparam.poseVector[i+Bspparam.k +1] - Bspparam.poseVector[i+2];
            double dt1 = fabs((Bspparam.k - 1)*(vts[i+1]->x - vts[i]->x)/Bspparam.amax);
            double dt2 = fabs((Bspparam.k - 1)*(vts[i+1]->y - vts[i]->y)/Bspparam.amax);
            double dt3 = fabs((Bspparam.k - 1)*(vts[i+1]->z - vts[i]->z)/Bspparam.amax);
            double mindt = max(dt1,max(dt2,dt3));
            if(dt < mindt)
            {
                for(int j = i+Bspparam.k+1;j < Bspparam.poseVector.size();++j)
                {
                    Bspparam.poseVector[j] += mindt - dt;
                }
            }

        }
    }
}

//Bspline由控制点计算位置
void cal_bspline(vector<Point*>path,vector<Point*>& route,int k,vector<double> posevector)
{
    int n = path.size();
    double step = 0.01;
    for(double t = posevector[k];t <= posevector[n]-step;t += step)
    {
        Point pt(cal_t(t,n,path,k,posevector));
        Point* point = new Point(pt.x,pt.y,pt.z,t);
        route.push_back(point);
    }
}

//根据pt计算每个点
Point cal_t(double t,int n,vector<Point*> path,int k,vector<double> posevector)
{
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    for(int i = 1;i<= n;++i )
    {
        px += path[i-1]->x*cal_Basic(i,t,k,posevector);
        py += path[i-1]->y*cal_Basic(i,t,k,posevector);
        pz += path[i-1]->z*cal_Basic(i,t,k,posevector);
    }
    return Point(px,py,pz);
}
//计算 B样条的基
double cal_Basic(int i,double t,int k,vector<double> posevector)
{
    double result;
    if (k == 0)
    {
        if(posevector[i-1]<=t && t<posevector[i])
            result = 1.0;
        else 
            result = 0.0;
    }
    else
    {
        double alpha,beta;
        double length1 = posevector[i-1+k] - posevector[i-1];
        double length2 = posevector[i+k] -posevector[i];
        if(length1 ==0)
            alpha = 0.0;
        else
            alpha = (t - posevector[i-1])/length1;
        if(length2 == 0)
            beta = 0.0;
        else
            beta = (posevector[i+k]-t)/length2;
        result = alpha*cal_Basic(i,t,k-1,posevector) + beta*cal_Basic(i+1,t,k-1,posevector);
    }
    return result;
}

//计算速度加速度控制点
void cal_vtats(vector<Point*> path,vector<Point*>& vts,vector<Point*>& ats)
{
    // Eigen::Vector3d p1;
    // Eigen::Vector3d p2;
    // Eigen::Vector3d v;
    // for(int i = 0;i<(int)path.size()-1;++i)
    // {
    //     p1<<path[i+1]->x,path[i+1]->y,path[i+1]->z;
    //     p2<<path[i]->x,path[i]->y,path[i]->z;
    //     v <<Bspparam.k*(p1-p2)/(Bspparam.poseVector[i+Bspparam.k+1]-Bspparam.poseVector[i+1]);
    //     Point* vt = new Point(v(0),v(1),v(2));
    //     vts.push_back(vt);
    // }
    // Eigen::Vector3d v1;
    // Eigen::Vector3d v2;
    // Eigen::Vector3d a;
    // for(int i = 0;i<(int)vts.size()-1;++i)
    // {
    //     v1<<vts[i+1]->x,vts[i+1]->y,vts[i+1]->z;
    //     v2<<vts[i]->x,vts[i]->y,vts[i]->z;
    //     a<<(Bspparam.k-1)*(v1-v2)/(Bspparam.poseVector[i+Bspparam.k+1]-Bspparam.poseVector[i+2]);
    //     Point* at = new Point(a(0),a(1),a(2));
    //     ats.push_back(at);
    // }

    double vx, vy, vz;
    for (int i = 0; i < path.size() -1; ++i)
    {
        vx =  Bspparam.k * (path[i+1]->x - path[i]->x) / (Bspparam.poseVector[i+Bspparam.k+1] - Bspparam.poseVector[i+1]);
        vy =  Bspparam.k * (path[i+1]->y - path[i]->y) / (Bspparam.poseVector[i+Bspparam.k+1] - Bspparam.poseVector[i+1]);
        vz =  Bspparam.k * (path[i+1]->z - path[i]->z) / (Bspparam.poseVector[i+Bspparam.k+1] - Bspparam.poseVector[i+1]);
        Point* vt = new Point(vx,vy,vz);
        vts.push_back(vt);
    }
    cout<<"vts:"<<vts.size()<<endl;
    double ax,ay,az;
    for (int  i = 0; i < vts.size()-1; ++i)
    {
        ax = (Bspparam.k - 1)*(vts[i+1]->x - vts[i]->x) / (Bspparam.poseVector[i+Bspparam.k+1] - Bspparam.poseVector[i+2]);
        ay = (Bspparam.k - 1)*(vts[i+1]->y - vts[i]->y) / (Bspparam.poseVector[i+Bspparam.k+1] - Bspparam.poseVector[i+2]);
        az = (Bspparam.k - 1)*(vts[i+1]->z - vts[i]->z) / (Bspparam.poseVector[i+Bspparam.k+1] - Bspparam.poseVector[i+2]);
        Point* at = new Point(ax,ay,az);
        ats.push_back(at);
    }
    cout<<"ats:"<<ats.size()<<endl;


}
