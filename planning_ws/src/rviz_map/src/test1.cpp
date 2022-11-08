#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "iostream"
#include "fstream"

using namespace std;

string  file_name1="/home/tt/planning_ws/src/rviz_map/path.csv";

void mapCallback(const nav_msgs::OccupancyGridConstPtr& map){
        cout<<"map resolution:"<<map->info.resolution<<endl;
        cout<<"map width:"<<map->info.width<<endl;
        cout<<"map height:"<<map->info.height<<endl;
        float origin_x= map->info.origin.position.x;
        float origin_y= map->info.origin.position.y;
        float origin_z= map->info.origin.position.z;
        cout<<"origin_x:"<<origin_x<<"\n"<<"origin_y"<<origin_y<<"\n"<< "origin_z"<<origin_z<<endl;
        float orientation_x=map->info.origin.orientation.x;
        float orientation_y=map->info.origin.orientation.y;
        float orientation_z=map->info.origin.orientation.z;
        float orientation_w=map->info.origin.orientation.w;
        cout<<"orientation_x"<<orientation_x<<"\n"<<"orientation_y"<<orientation_y<<"\n"<<"orientation_z"
        <<orientation_z<<"orientation_w"<<orientation_w<<endl;
      ofstream file(file_name1);
            if(!file){
                cout<<"can not find file_name1!"<<endl;
                return;
            }else{
                for(int i=0;i<map->data.size();++i){
                    file<<int(map->data[i])<<"\n";
                }
            }
            file.close();
            return ;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"get_map");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/map",10,mapCallback);
    ros::spin();
    return 0;
}

