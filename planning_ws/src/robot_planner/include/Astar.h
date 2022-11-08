#ifndef   ASTAR_H
#define  ASTAR_H
#include<iostream>
#include <queue>
#include <vector>
#include <stack>
#include <algorithm>
#include <cstdlib>
#include<math.h>
using namespace std;

typedef struct Node
{
        int x,y;
        int g;
        int h;
        int f;
        Node* father;

        Node(int x, int y)
        {
            this->x = x;
            this->y = y;
            this->g = 0;
            this->h = 0;
            this->f  = 0;
            this->father = nullptr;
        }

        Node(int x, int y, Node* father)
        {
            this->x = x;
            this->y = y;
            this->g = 0;
            this->h = 0;
            this->f  = 0;
            this->father = father;
        }
};

class Astar{
public:
    Astar(Node* startPos, Node* endPos, int** map, int sizex, int sizey);
    ~Astar();
    vector<Node*> search();
    void checkPoint(int x, int y, Node* father, int g);
    void NextStep(Node* currentPoint);
    int isContains(vector<Node*>* Nodelist, int x, int y);
    void countGHF(Node* sNode, Node* eNode, int g);
    static bool compare(Node* n1, Node* n2);
    bool isObs(int x, int y);
    bool isInvalid(int x, int y);
    void printPath(Node* current);
    void printMap();

    Node *startPos;
    Node *endPos;

    vector<Node*> openList;
    vector<Node*> closeList;
    vector<Node*> path;
    
    int sizex = 40;
    int sizey = 40;

    int** pMap;
    };
#endif