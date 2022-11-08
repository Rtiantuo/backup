#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "iostream"
#include "fstream"

using namespace std;

string  file_name1="/home/tt/ctakin_ws/src/map_test/path1.csv";

void mapCallback(const nav_msgs::PathConstPtr& path){
    cout<<"start write!"<<endl;
    cout<<"path of  size:"<<path->poses.size()<<endl;
    ofstream file(file_name1);
            if(!file){
                cout<<"can not find file_name1!"<<endl;
                return;
            }else{
                for(int i=0;i<path->poses.size();++i){
                    file<<path->poses[i].pose.position.x<<" "<<path->poses[i].pose.position.y<<"\n";
                    cout<<path->poses[i].pose.position.x<<"\t"<<path->poses[i].pose.position.y<<"\n";
                }
            }
            cout<<"all write!"<<endl;
            file.close();
           return ;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"get_path");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/move_base/TebLocalPlannerROS/global_plan",10,mapCallback);
    ros::spin();
    cout<<"hello"<<endl;
    return 0;
}


