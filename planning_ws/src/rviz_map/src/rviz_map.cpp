#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
 
 
 int main(int argc,char **argv)
{
    ros::init(argc,argv,"grid_cell");
    ros::NodeHandle nh;
    ros::Publisher pub;
    nav_msgs::GridCells cells;
 
    cells.header.frame_id=" ";
    cells.cell_height=0.3;
    cells.cell_width=0.3;
    cells.cells.resize(3);
    cells.cells[0].x=1;
    cells.cells[0].y=1;
    cells.cells[0].z=0;
 
    pub = nh.advertise<nav_msgs::GridCells>("/cells", 1);
 
 
    while (ros::ok())
    {
        pub.publish(cells);
    }
}