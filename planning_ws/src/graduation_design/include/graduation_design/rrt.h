#ifndef _RRT_H_
#define _RRT_H_

#include <Eigen/Eigen>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

using namespace std;    
using namespace Eigen;

typedef struct rrtode{
    double x,y;
    int preInd = -1;
    // rrtNode(){}
    // rrtNode(double x,double y):double(x),double(y),preInd(-1) {}
}rrtNode;

class RRT{
    public:
        // <vector>rrtNode rrtTree;
        vector<rrtNode> initRRT(rrtNode start);
        rrtNode generateTempPoint();//随机生成一个点
        int nearestT(rrtNode randpoint,vector<rrtNode> rrtTree);//找到离采样点最近的点
        double getxyzDistance(rrtNode node1,rrtNode node2);//计算距离
        rrtNode Steer(rrtNode randpoint,int index,vector<rrtNode> rrtTree);//扩展
        bool collision_detection(rrtNode newpoint,rrtNode startpoint);//碰撞检测
        bool ObstacleFree(rrtNode node);//某一个点是否有障碍物
        vector<rrtNode> rrt_search(rrtNode rrt_start,rrtNode rrt_goal);//rrt路径搜索算法
};

#endif