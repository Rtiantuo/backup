#include"global_plan/test_astar.h"

astar2d::astar2d(int** pmap,int sizex,int sizey)
{
    this->pMap = pmap;
    this->sizex = sizex;
    this->sizey = sizey;
}

astar2d::~astar2d()
{}

void astar2d::search(Eigen::Vector2d start,Eigen::Vector2d goal)
{
    Node* cur_node = new Node();
    cur_node->parent = nullptr;
    cur_node->position = start;
    cur_node->index = posToIndex(start);
    cur_node->g_score = 0.0;
    cur_node->f_score = getDiagHeu(start,goal);
    cur_node->node_state = IN_OPEN_SET;

    open_set.push(cur_node);
    expand_nodes[cur_node->index] = cur_node;

    while(!open_set.empty())
    {
        cur_node = open_set.top();
        cur_node->node_state = IN_CLOSE_SET;
        open_set.pop();

        // 判断是否到达
        bool reach_end = abs(cur_node->position(0)-goal(0))<0.1&&abs(cur_node->position(1)-goal(1))<0.1;
        if(reach_end)
        {
            retrievePath(cur_node);
            break;
        }

        Eigen::Vector2d cur_pos = cur_node->position;
        Eigen::Vector2d pro_pos;
        Eigen::Vector2d d_pos;
        //对当前节点进行扩展
        for(int dx = -1;dx<=1;dx = dx+1)
        {
            for(int dy = -1;dy<=1;dy = dy+1)
            {
                d_pos<<dx,dy;
                //去除当前节点，因为当前节点已经扩展过了
                if(dx==0&&dy==0)
                {
                    continue;
                }
                pro_pos = cur_pos+d_pos;
                 //排除障碍物的点
                if(isobs(pro_pos ))
                {
                    continue;
                }
                //计算当前扩展点的cost
                double tmp_g_score,tmp_f_score;
                tmp_g_score = d_pos.squaredNorm()+cur_node->g_score;
                //cout<<"  g_score: "<<tmp_g_score<<endl;
                tmp_f_score = tmp_g_score+getDiagHeu(pro_pos,goal);
                //cout<<" h_score:"<<getDiagHeu(pro_pos,goal)<<endl;

                //判断当前扩展节点的状态
                int idx = posToIndex(pro_pos);
                Node* pro_node = nullptr;

                //如果在close list中找到，即在expanded中有，即为close，continue
                if(expand_nodes.find(idx)!=expand_nodes.end())
                {  
                    pro_node = expand_nodes[idx];
                }

                //如果没找到，即代表该节点没有被扩展  追加到openLIST
                if (pro_node == nullptr)
                {
                    pro_node = new Node();
                    pro_node->parent = cur_node;
                    pro_node->position = pro_pos;
                    pro_node->index = posToIndex(pro_pos);
                    pro_node->g_score = tmp_g_score;
                    pro_node->f_score = tmp_f_score;
                    pro_node->node_state = IN_OPEN_SET;
                    open_set.push(pro_node);
                    expand_nodes[pro_node->index] = pro_node;
                    continue;
                }

                //如果找到且为close，则该节点不需要操作
                if(pro_node->node_state == IN_CLOSE_SET)
                {
                    continue;
                }

                //如果找到为open，则考虑是否更新g
                if (pro_node->node_state == IN_OPEN_SET)
                {
                    if(tmp_g_score<pro_node->g_score)
                    {
                        pro_node->g_score = tmp_g_score;
                        pro_node->f_score = tmp_f_score;
                        pro_node->parent = cur_node;
                    }
                    continue;
                }
            }
        }

    }
    

}


int astar2d::posToIndex(Eigen::Vector2d pt)
{
    int value1 = pt(0);   //x
    int value2  = pt(1);  //y
    return value2*sizex+value1;
}

int astar2d::getDiagHeu(Eigen::Vector2d x1,Eigen::Vector2d x2)
{
    Eigen::Vector2d dis;
    int dx = fabs(x1(0)-x2(0));
    int dy = fabs(x1(1)-x2(1));
    dis<<dx,dy;
    //double heu=dis.squaredNorm();
     /*double minv = min(dx,dy);
     double maxv = dx+dy-minv;
     double heu = minv*1.414+(maxv-minv);*/
     int heu = dx+ dy;
    return heu;
}

bool astar2d::isobs(Eigen::Vector2d pos)
{
    int idx = pos(0);
    int idy = pos(1);

    if(idx<0||idy<0||idx>=sizex||idy>=sizey)
    {
        return true;
    }

    if(this->pMap[idx][idy]==1)
    {
        return true;
    }
    return false;
}

void astar2d::retrievePath(Node* end_node)
{
    Node* cur_node = end_node;
    path_nodes.push_back(cur_node);

    while (cur_node->parent!=nullptr)
    {
        cur_node = cur_node->parent;
        path_nodes.push_back(cur_node);
    }

    reverse(path_nodes.begin(),path_nodes.end());
}

void astar2d::reset()
{
    for(auto it = expand_nodes.begin(); it != expand_nodes.end(); ++it)
    {
        delete it->second;
        it->second=nullptr;
    }
    expand_nodes.clear();
    path_nodes.clear();
    open_set.empty();

    priority_queue<Node*,vector<Node*>,cmp>empty_queue;
    open_set.swap(empty_queue);
}



