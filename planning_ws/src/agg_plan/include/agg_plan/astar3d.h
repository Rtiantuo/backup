#ifndef   ASTAR_H
#define  ASTAR_H
#include<iostream>
#include <queue>
#include <vector>
#include <stack>
#include <algorithm>
#include <cstdlib>
#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Dense>
#include <Eigen/StdVector>
using namespace std;





typedef struct Node
{
        int x,y,z;
        int g;
        int h;
        int f;
        Node* father;

        Node(int x, int y,int z)
        {
            this->x = x;
            this->y = y;
            this->z =  z;
            this->g = 0;
            this->h = 0;
            this->f  = 0;
            this->father = nullptr;
        }

        Node(int x, int y, int z,Node* father)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            this->g = 0;
            this->h = 0;
            this->f  = 0;
            this->father = father;
        }
};


class Astar{
public:
    Astar(Node* startPos, Node* endPos, int* map, int sizex, int sizey, int sizez);
    //~Astar();
    vector<Node*> search();
    void checkPoint(int x, int y, int z, Node* father, int g);
    void NextStep(Node* currentPoint);
    int isContains(vector<Node*>* Nodelist, int x, int y,int z);
    void countGHF(Node* sNode, Node* eNode, int g);
    static bool compare(Node* n1, Node* n2);
    
    bool noTo(Eigen::Vector3d pt);
    bool iscollision(Eigen::Vector3d startp,Eigen::Vector3d goalp);
    bool isObs(int x, int y,int z);
    bool isInvalid(int x, int y,int z);
    void printPath(Node* current);
    void printMap();
    int gridindex2Lineindex(int x,int y,int z);
    vector<Node*> cutPath(vector<Node*>& path);

    Node *startPos;
    Node *endPos;

    vector<Node*> openList;
    vector<Node*> closeList;
    vector<Node*> path;
    
    int sizex;
    int sizey;
    int sizez;

    int*** pMap;
    };
#endif