#ifndef ASTAR2D_H
#define ASTAR2D_H
#include<iostream>
#include<queue>
#include<vector>
#include<stack>
#include<algorithm>
#include<cstdlib>
#include<math.h>
using  namespace std;

struct Node
{
    float x,y;
    float g;
    float h;
    float f;
    Node* father;

    Node(float x,float y)
    {
        this->x=x;
        this->y=y;
        this->g=0;
        this->h=0;
        this->f=0;
        this->father=nullptr;
    }
    Node(float x,float y,Node* father)
    {
        this->x=x;
        this->y=y;
        this->g=0;
        this->h=0;
        this->f=0;
        this->father=father;
    }
};

class Astar{
public:
    Astar(Node* startPos, Node* endPos, int* map, int sizex, int sizey,int sizez);
    //~Astar();
    vector<Node*> search();
    void checkPoint(float x, float y, Node* father);
    void NextStep(Node* currentPoint);
    int isContains(vector<Node*>* Nodelist, int x, int y);
    void countGHF(Node* sNode, Node* eNode);
    static bool compare(Node* n1, Node* n2);
    
    bool isObs(int x, int y);
    bool isInvalid(int x, int y);
    void printPath(Node* current);
    int gridindex2Lineindex(int x,int y);

    Node *startPos;
    Node *endPos;

    vector<Node*> openList;
    vector<Node*> closeList;
    vector<Node*> path;
    
    int sizex = 40;
    int sizey = 40;
    int sizez=40;

    int step=2;
    float sin30Len=step*sin(M_PI/6);
    float cos30Len=step*cos(M_PI/6);

    int** pMap;
    };
#endif