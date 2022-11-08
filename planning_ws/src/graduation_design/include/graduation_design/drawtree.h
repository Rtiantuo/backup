#ifndef DRAWTREE_H
#define DRAWTREE_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

//defining markers
visualization_msgs::Marker startPoint;
visualization_msgs::Marker goalPoint;
visualization_msgs::Marker newPoint;
visualization_msgs::Marker rrtTreeMarker;
visualization_msgs::Marker finalPath;

ros::Publisher treeMarker_pub ;
ros::Publisher goal_pub ;

void initializeMarkers(visualization_msgs::Marker &startPoint, visualization_msgs::Marker &goalPoint, 
                        visualization_msgs::Marker &randomPoint, visualization_msgs::Marker &rrtTreeMarker, 
                        visualization_msgs::Marker &finalPath)
{
    //init headers
    startPoint.header.frame_id    = goalPoint.header.frame_id    = newPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = "map";
    startPoint.header.stamp       = goalPoint.header.stamp       = newPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
    startPoint.ns                 = goalPoint.ns                 = newPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = "map";
    startPoint.action             = goalPoint.action             = newPoint.action             = rrtTreeMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
    startPoint.pose.orientation.w = goalPoint.pose.orientation.w = newPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;
    startPoint.lifetime           = goalPoint.lifetime           = newPoint.lifetime           = rrtTreeMarker.lifetime           = finalPath.lifetime           = ros::Duration(0);

    //setting id for each marker
    startPoint.id     = 0;
	goalPoint.id      = 1;
	newPoint.id    = 2;
	rrtTreeMarker.id  = 3;
    finalPath.id      = 4;

    //defining types
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	startPoint.type  = goalPoint.type = newPoint.type = visualization_msgs::Marker::SPHERE;

    //setting scale
	rrtTreeMarker.scale.x = 0.5;
	finalPath.scale.x     = 0.5;
	startPoint.scale.x   = goalPoint.scale.x = newPoint.scale.x = 1;
    startPoint.scale.y   = goalPoint.scale.y = newPoint.scale.y = 1;
    startPoint.scale.z   = goalPoint.scale.z = newPoint.scale.z = 1;

    //assigning colors
	startPoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
    newPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.4f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 0.7f;
    startPoint.color.a = goalPoint.color.a = newPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
}

#endif
