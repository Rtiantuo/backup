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

//节点信息
struct Node
{
    int index;
    Eigen::Vector2d position;  //位置信息
    double g_score,f_score;  //启发式函数值
    Node* parent;  //父节点
    char node_state;  //节点状态

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
        return a->f_score>b->f_score;    //小顶堆  --open_list中按f值从小到大排序
    }
};

class astar2d{
public:
    astar2d(int** pmap,int sizex,int sizey);
    ~astar2d();

    int sizex = 40;
    int sizey = 40;
    int** pMap;

    std::vector<Node*>path_nodes;  //储存得到的路径点

    priority_queue<Node*,vector<Node*>,cmp>open_set;     //将f按升序排列
    unordered_map<int,Node*>expand_nodes;

    void search(Eigen::Vector2d start,Eigen::Vector2d goal);
    //pose转线性索引
    int posToIndex(Eigen::Vector2d pt);  
    //计算h_score
    int getDiagHeu(Eigen::Vector2d x1,Eigen::Vector2d x2);
    bool isobs(Eigen::Vector2d pos);
    //路径回溯
    void retrievePath(Node* end_node);
    void reset();
};
#endif