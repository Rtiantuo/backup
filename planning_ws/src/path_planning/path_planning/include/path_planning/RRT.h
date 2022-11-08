#ifndef RRT_H
#define RRT_H
#include<iostream>
#include<vector>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include <Eigen/StdVector>

using namespace std;

struct Node{
    Eigen::Vector2d position;
    Eigen::Vector2d prePosition;
    int preId;

    Node(){};
    ~Node(){};
};

class RRT{
    public:
        RRT();
        RRT(Eigen::Vector2d start,Eigen::Vector2d goal,int step);
        ~RRT();
        int FindNearest(Eigen::Vector2d samPoint);
        Eigen::Vector2d newPoint(Eigen::Vector2d samPoint,Eigen::Vector2d neraestPoint);
        bool checkPoint(int** pmap,Eigen::Vector2d newPt,Eigen::Vector2d nearestPoint);
        bool isCollision(int** pmap,Eigen::Vector2d newPt);
        void addTree(Eigen::Vector2d subPt,int index);
        void retrivePath();

        vector<Node*> path;
        vector<Node*> Tree;
        int step;
        Eigen::Vector2d start;
        Eigen::Vector2d goal;
};

#endif