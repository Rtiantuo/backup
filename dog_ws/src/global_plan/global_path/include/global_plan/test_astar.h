#ifndef ASTAR2D_H
#define ASATR2D_H

#include<iostream>
#include<queue>
#include<vector>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include<unordered_map>

using namespace std;

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1>>30

struct Node
{
    int index;
    Eigen::Vector2d position;
    double g_score,f_score;
    Node* parent;
    char node_state;

    Node()
    {
        parent = NULL;
        node_state=NOT_EXPAND;
    }

    ~Node()
    {}
};


struct cmp
{
    bool operator()(const Node *a,const Node *b)
    {
        return a->f_score>b->f_score;    //小顶堆
    }
};

class astar2d{
public:
    astar2d(int** pmap,int sizex,int sizey);
    ~astar2d();

    int sizex = 40;
    int sizey = 40;
    int** pMap;

    std::vector<Node*>path_nodes;

    priority_queue<Node*,vector<Node*>,cmp>open_set;     //将f按升序排列
    unordered_map<int,Node*>expand_nodes;

    void search(Eigen::Vector2d start,Eigen::Vector2d goal);
    int posToIndex(Eigen::Vector2d pt);  
    int getDiagHeu(Eigen::Vector2d x1,Eigen::Vector2d x2);
    bool isobs(Eigen::Vector2d pos);
    void retrievePath(Node* end_node);
    void reset();
};
#endif