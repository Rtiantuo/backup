#include <ros/ros.h>
#include "PolyClosedForm_MinimumSnap.h"

#include <Eigen/Eigen>
#include <vector>


#include <visualization_msgs/Marker.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher traj_pub;

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;
    int _poly_num1D = 8;
    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

void traj_publish(MatrixXd poly,VectorXd time)
{
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";

    geometry_msgs::PoseStamped pt;
    pt.pose.orientation.x = 0;
    pt.pose.orientation.y = 0;
    pt.pose.orientation.z = 0;
    pt.pose.orientation.w = 1;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = "map";

    Vector3d pos;
    path.poses.clear();
    for (int i = 0;i < time.size(); i++ )
    {
        for (double t = 0.0; t < time(i); t += 0.01)
        {
            pos = getPosPoly(poly, i, t);
            pt.pose.position.x = pos(0);
            pt.pose.position.y= pos(1);
            pt.pose.position.z= pos(2);
            path.poses.push_back(pt);
        }
    }
    traj_pub.publish(path);
}

void points_callback(const visualization_msgs::MarkerConstPtr& msg)
{
    // 将消息存进路径点的数组

    int len = msg->points.size()/3;
    cout<<"len"<<len<<endl;
    int a=0;

    MatrixXd Path = MatrixXd::Zero(len,3);

    for (int i = 0;i<len-1;i++)
    {
        cout<<"a "<<a<<endl;
        Path.row(i) = Vector3d(msg->points[a].x,msg->points[a].y,msg->points[a].z);

        a +=3;
    }

    Path.row(len-1) = Vector3d(msg->points[msg->points.size()-1].x,msg->points[msg->points.size()-1].y,msg->points[msg->points.size()-1].z);
    cout<<Path<<endl;

    PolyClosedForm_MinimumSnap Close;

    VectorXd Time = Close.timeAllocation(Path);
    cout<<"Time"<<Time<<endl;
    int d_order = 4;
    MatrixXd Vel = MatrixXd::Zero(2,3);
    MatrixXd Acc = MatrixXd::Zero(2,3);
    MatrixXd poly = Close.PolyQPGeneration(d_order,Path,Vel,Acc,Time);
    // cout<<"poly"<<poly<<endl;
    traj_publish(poly,Time);
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"trajectory_generation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("visualization_marker",10,points_callback);

    // 发布生成的轨迹
    traj_pub = n.advertise<nav_msgs::Path>("/trajectory",1,true);
    ros::spin();
    return 0;
}