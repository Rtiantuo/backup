#include<agg_plan/astar3d.h>

//将地图载入的时候，只载入start以上sizez的范围
int map_start = 3;

//--地图数据初始化
Astar::Astar(Node* startPos,Node* endPos,int * map,int sizex,int sizey,int sizez)
{
    this->startPos=startPos;
    this->endPos=endPos;
    this->sizex=sizex;
    this->sizey=sizey;
    this->sizez=sizez;

    //三维数组
    pMap=(int***)malloc(sizex*sizeof(int*));
    for(int i=0;i<sizex;++i)
    {
        pMap[i]=(int**)malloc(sizey*sizeof(int*));
    }
    for(int i=0;i<sizex;++i)
    {
        for(int j=0;j<sizey;++j)
        {
            pMap[i][j]=(int*)malloc(sizez*sizeof(int));
        }
    }
    //没有膨胀
    int index;
    for(int x=0;x<sizex;++x)
    {
        for(int y=0;y<sizey;++y)
        {
            for(int z=0;z<sizez;++z)
            {
                pMap[x][y][z]=0;
                //在进行路径规划时候，只取实际地图的第四层(0.4m)到第四层到sizez+4,即(0.1*sizez+0.4)m
                index=gridindex2Lineindex(x,y,z + map_start);
                if(map[index]>90)
                {
                    pMap[x][y][z]=1;               
                }
            }
        }
    }


}

int Astar::gridindex2Lineindex(int x,int y,int z)
{
    int LineIndex=z*sizex*sizey+y*sizex+x;
    return LineIndex;
}

vector<Node*> Astar::search()
{
    cout<<"start search!"<<endl;
    if(isObs(startPos->x,startPos->y,startPos->z)||isObs(endPos->x,endPos->y,endPos->z)) return path;
    Node* current;
    openList.push_back(startPos);

    while(openList.size()>0)
    {
        current=openList[0];
        if(current->x==endPos->x && current->y==endPos->y &&current->z==endPos->z)
        {
            cout<<"find the path!"<<endl;
            printPath(current);
            break;
        }
        NextStep(current);
        closeList.push_back(current);
        openList.erase(openList.begin());
        sort(openList.begin(),openList.end(),compare);
    }
    return path;
}

void Astar::NextStep(Node* currentPoint)
{
    checkPoint(currentPoint->x-1,currentPoint->y-1,currentPoint->z+1,currentPoint,17);
    checkPoint(currentPoint->x-1,currentPoint->y,currentPoint->z+1,currentPoint,14);
    checkPoint(currentPoint->x-1,currentPoint->y+1,currentPoint->z+1,currentPoint,17);
    checkPoint(currentPoint->x,currentPoint->y-1,currentPoint->z+1,currentPoint,14);
    checkPoint(currentPoint->x,currentPoint->y,currentPoint->z+1,currentPoint,10);
    checkPoint(currentPoint->x,currentPoint->y+1,currentPoint->z+1,currentPoint,14);
    checkPoint(currentPoint->x+1,currentPoint->y-1,currentPoint->z+1,currentPoint,17);
    checkPoint(currentPoint->x+1,currentPoint->y,currentPoint->z+1,currentPoint,14);
    checkPoint(currentPoint->x+1,currentPoint->y+1,currentPoint->z+1,currentPoint,17);

    checkPoint(currentPoint->x-1,currentPoint->y-1,currentPoint->z,currentPoint,14);
    checkPoint(currentPoint->x-1,currentPoint->y,currentPoint->z,currentPoint,10);
    checkPoint(currentPoint->x-1,currentPoint->y+1,currentPoint->z,currentPoint,14);
    checkPoint(currentPoint->x,currentPoint->y-1,currentPoint->z,currentPoint,10);
    checkPoint(currentPoint->x,currentPoint->y+1,currentPoint->z,currentPoint,10);
    checkPoint(currentPoint->x+1,currentPoint->y-1,currentPoint->z,currentPoint,14);
    checkPoint(currentPoint->x+1,currentPoint->y,currentPoint->z,currentPoint,10);
    checkPoint(currentPoint->x+1,currentPoint->y+1,currentPoint->z,currentPoint,14);

    checkPoint(currentPoint->x-1,currentPoint->y-1,currentPoint->z-1,currentPoint,17);
    checkPoint(currentPoint->x-1,currentPoint->y,currentPoint->z-1,currentPoint,14);
    checkPoint(currentPoint->x-1,currentPoint->y+1,currentPoint->z-1,currentPoint,17);
    checkPoint(currentPoint->x,currentPoint->y-1,currentPoint->z-1,currentPoint,14);
    checkPoint(currentPoint->x,currentPoint->y,currentPoint->z-1,currentPoint,10);
    checkPoint(currentPoint->x,currentPoint->y+1,currentPoint->z-1,currentPoint,14);
    checkPoint(currentPoint->x+1,currentPoint->y-1,currentPoint->z-1,currentPoint,17);
    checkPoint(currentPoint->x+1,currentPoint->y,currentPoint->z-1,currentPoint,14);
    checkPoint(currentPoint->x+1,currentPoint->y+1,currentPoint->z-1,currentPoint,17);
}

void Astar::checkPoint(int x,int y,int z,Node* father,int g)
{
    if(isInvalid(x,y,z))  return;
    if(isObs(x,y,z))  return;
    if(isContains(&closeList,x,y,z)!=-1) return;

    int index;
    if((index=isContains(&openList,x,y,z))!=-1)
    {
        //--在openlist中
        Node* point=openList[index];
        //--更新g
        if(point->g>(father->g+g))
        {
            point->father=father;
            point->g=father->g+g;
            point->f=point->g+point->h;
        }
    }
    else
    {
        Node* point=new Node(x,y,z,father);
        countGHF(point,endPos,g);
        openList.push_back(point);
    }
}

bool Astar::isInvalid(int x,int y,int z)
{
    if(x<0 || x>sizex || y<0 || y>sizey || z<0 || z>sizez)
    {
        return true;
    }
    return false;
}

bool Astar::isObs(int x,int y,int z)
{
    if(pMap[x][y][z]==1)
    {
        return true;
    }
    return false;
}
vector<Node*> Astar::cutPath(vector<Node*>& path)
{
    vector<Node*> cutpath;
    Node* pt = new Node(path[0]->x,path[0]->y,path[0]->z);
    cutpath.push_back(pt);
    Eigen::Vector3d startp(path[0]->x,path[0]->y,path[0]->z);
    for(int i = 1;i<(int)path.size();++i)
    {
        Eigen::Vector3d goalp(path[i]->x,path[i]->y,path[i]->z);
        if(iscollision(startp,goalp))
        {
            Node* pt = new Node(path[i-1]->x,path[i-1]->y,path[i-1]->z);
            cutpath.push_back(pt);
            startp<<path[i-1]->x,path[i-1]->y,path[i-1]->z;
        }
    }
        int n = path.size() - 1;
        Node* pt1 = new Node(path[n]->x,path[n]->y,path[n]->z);
        cutpath.push_back(pt1);
    return cutpath;
}

bool Astar::iscollision(Eigen::Vector3d startp,Eigen::Vector3d goalp)
{
    Eigen::Vector3d dirVec = goalp - startp;
    double dis = dirVec.norm();
    dirVec.normalize();
    Eigen::Vector3d checkPt;
    for(int i =0;i<dis;++i)
    {
        checkPt = startp + i*dirVec;
        if(noTo(checkPt)) return true;
    }
    return false;
}

bool Astar::noTo(Eigen::Vector3d pt)
{
    int x1 = floor(pt(0));
    int x2 = ceil(pt(0));
    int y1 = floor(pt(1));
    int y2 = ceil(pt(1));
    int z1 = floor(pt(2));
    int z2 = ceil(pt(2));
    if(pMap[x1][y1][z1]==1 || pMap[x1][y1][z2]==1 || pMap[x1][y2][z1]==1 || pMap[x1][y2][z2]==1 || pMap[x2][y1][z1]==1 || pMap[x2][y1][z2]==1 || pMap[x2][y2][z1]==1 || pMap[x2][y2][z2]==1)  return true;
    return false;
    }

int Astar::isContains(vector<Node*>* Nodelist,int x,int y,int z)
{
    for(int i=0;i<(int)Nodelist->size();++i)
    {
        if(Nodelist->at(i)->x==x && Nodelist->at(i)->y==y && Nodelist->at(i)->z==z)
        {
            return i;
        }
    }
    return -1;
}

void Astar::countGHF (Node* sNode,Node* eNode, int g)
{
    double h1=abs(sNode->x-eNode->x)*10;
    double h2=abs(sNode->y-eNode->y)*10;
    double h3=abs(sNode->z-eNode->z)*10;
    double h=h1+h2+h3;
    double currentG=sNode->father->g+g;
    double f=currentG+h;
    sNode->f=f;
    sNode->h=h;
    sNode->g=currentG;
}

bool Astar::compare (Node* n1,Node* n2)
{
    return n1->f <n2->f;
}

void Astar::printPath(Node* current)
{
    Node* cur_node=current;
    path.push_back(cur_node);
    while(cur_node->father!=nullptr)
    {
        cur_node=cur_node->father;
        path.push_back(cur_node);
    }
    reverse(path.begin(),path.end());
}