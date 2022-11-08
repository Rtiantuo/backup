#include"path_planning/RRT.h"
#include<iostream>
#include<vector>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include <Eigen/StdVector>

//初始化
RRT::RRT(Eigen::Vector2d startPoint,Eigen::Vector2d goalPoint,int stepSize)
{
    this->start=startPoint;
    this->goal=goalPoint;
    this->step=stepSize;

    Node* cur_node= new Node();
    cur_node->position=start;
    cur_node->preId=-1;
    Tree.push_back(cur_node);
}
RRT::RRT(){};

RRT::~RRT(){};

//查找当前采样点的最近点
int RRT::FindNearest(Eigen::Vector2d samPoint)
{
    int index = 0;
    int mindist = 999;
    for(int i =0;i<(int)Tree.size();++i)
    {
        int dist = (samPoint - Tree[i]->position).norm();
        if (dist<mindist)
        {
            mindist = dist;
            index = i;
        }
    }
    return index;
}

//生成新树节点
Eigen::Vector2d RRT::newPoint(Eigen::Vector2d samPoint,Eigen::Vector2d neraestPoint)
{
    Eigen::Vector2d dirVec = samPoint-neraestPoint;
    dirVec.normalize();
    Eigen::Vector2d newPt =neraestPoint+ step*dirVec;
    return newPt;
}

//判断连上的节点是否有效
bool RRT::checkPoint(int** pmap,Eigen::Vector2d newPt,Eigen::Vector2d nearestPoint)
{
    Eigen::Vector2d dirVec = (newPt-nearestPoint);
    dirVec.normalize();
    double n=0.5;
    if(isCollision(pmap,newPt))  return false;
    Eigen::Vector2d checkPt =nearestPoint; 
    for(double i=0;i<=step;i+=n)
    {
        checkPt = nearestPoint+i*dirVec;
        if(isCollision(pmap,checkPt))  return false;
    }
    return true;
}

bool RRT::isCollision(int** pmap,Eigen::Vector2d newPt)
{
    int x1 = floor(newPt(0));
    int x2 = ceil(newPt(0));
    int y1 = floor(newPt(1));
    int y2 = ceil(newPt(1));
    if(x1<=0 || x1>=100 || x2<=0 || x2>=100 || y1 <=0 || y1>=100 || y2<=0 || y2>=100)  return true;
    if(pmap[x1][y1]==1 || pmap[x1][y2]==1 || pmap[x2][y1]==1 || pmap[x2][y2]==1)  return true;
    return false;
}

//将子节添加到树中
void RRT::addTree(Eigen::Vector2d subPt,int index)
{
    Node* node= new Node();
    node->position=subPt;
    node->preId = index;
    Tree.push_back(node);
}

//路径回溯
void RRT::retrivePath()
{
    int ind = Tree.size()-1;
    Node* cur_node = Tree[ind];
    path.push_back(cur_node);
    while (true)
    {
        cur_node = Tree[cur_node->preId];
        path.push_back(cur_node);
        if(cur_node->preId == -1)  break;
    }
}


