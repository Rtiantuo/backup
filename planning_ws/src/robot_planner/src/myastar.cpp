#include "myastar.h"
myAstar::myAstar(int** pmap, int sizex, int sizey)
{
    this->pMap = pmap;
    this->sizex = sizex;
    this->sizey = sizey;
}

myAstar::~myAstar()
{

}

void myAstar::search(Eigen::Vector2d start, Eigen::Vector2d goal)
{
    Node* cur_node = new Node();
    cur_node->parent = nullptr;
    cur_node->position = start;
    cur_node->index = posToIndex(start);
    cur_node->g_score = 0.0;
    cur_node->f_score = getDiagHeu(start,goal);
    cur_node->node_state = IN_OPEN_SET;

    open_set.push(cur_node);
    expanded_nodes[cur_node->index]=cur_node;

    while(!open_set.empty())
    {
        cur_node = open_set.top();
        cur_node->node_state = IN_CLOSE_SET;
        open_set.pop();

        bool reach_end = abs(cur_node->position[0]-goal[0])<0.1 && abs(cur_node->position[1]-goal[1])<0.1;
        if (reach_end)
        {
            retrievePath(cur_node);
            break;
        }

        Eigen::Vector2d cur_pos = cur_node->position;
        Eigen::Vector2d pro_pos;
        Eigen::Vector2d d_pos;
        for (double dx = -1;dx <=1;dx=dx+1)
        {
            for(double dy=-1;dy<=1;dy=dy+1)
            {
                d_pos << dx, dy;
                if (dx==0 && dy==0)
                {
                    continue;
                }
                pro_pos = cur_pos + d_pos;
                //判断扩展节点是否为障碍物
                if (isobs(pro_pos))
                {
                    continue;
                }
                /* ---------- compute cost ---------- */
                double tmp_g_score, tmp_f_score;
                tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
                tmp_f_score = tmp_g_score + getDiagHeu(pro_pos, goal);

                //判断扩展节点状态
                int idx = posToIndex(pro_pos);
                Node* pro_node = nullptr;

                //如果找到，并且为close，continue
                if (expanded_nodes.find(idx) != expanded_nodes.end())
                {
                    pro_node = expanded_nodes[idx];
                }

                //如果没有找到，那么表示这个节点从来没有被扩展
                if(pro_node == nullptr)
                {
                    pro_node = new Node();
                    pro_node->parent = cur_node;
                    pro_node->position = pro_pos;
                    pro_node->index = posToIndex(pro_pos);
                    pro_node->g_score = tmp_g_score;
                    pro_node->f_score = tmp_f_score;
                    pro_node->node_state = IN_OPEN_SET;
                    open_set.push(pro_node);
                    expanded_nodes[pro_node->index]=pro_node;
                    continue;
                }
                //如果找到，并且为close，那么不需要操作
                if(pro_node->node_state == IN_CLOSE_SET)
                {
                    continue;
                }

                //如果找到，并且为open,那么考虑是否更新g值
                if(pro_node->node_state==IN_OPEN_SET)
                {
                    if (tmp_g_score<pro_node->g_score)
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

double myAstar::getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2) {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double minv = min(dx,dy);
    double maxv = dx+dy-minv;
    double heu = minv*1.414 + (maxv-minv);
    return 1.001*heu;
}

int myAstar::posToIndex(Eigen::Vector2d pt)
{
    int value1 = pt[0];
    int value2 = pt[1];
    return value2*sizex + value1;
}

bool myAstar::isobs(Eigen::Vector2d pos)
{
    int idx = pos[0];
    int idy = pos[1];

    if(idx<0 || idy<0 || idx>=sizex || idy>=sizey)
    {
        return true;
    }

    if (this->pMap[idx][idy]==1)
    {
        return true;
    }
    return false;
}

void myAstar::retrievePath(Node* end_node)
{
    Node* cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != nullptr) {
        cur_node = cur_node->parent;
        path_nodes_.push_back(cur_node);
     }

    reverse(path_nodes_.begin(), path_nodes_.end());
}

void myAstar::reset()
{
  expanded_nodes.clear();
  path_nodes_.clear();

  priority_queue<Node*,vector<Node*>,cmp>empty_queue;
  open_set.swap(empty_queue);
}