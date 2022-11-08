#include "Astar.h"
Astar::Astar(Node* startPos, Node* endPos, int** map, const int sizex, const int sizey)
{
    //起始点,终点
    this->startPos = startPos ;
    this->endPos = endPos;

    //大小
    this->sizex = sizex;
    this->sizey = sizey;

    //为pMap分配大小
    this->pMap = (int **)malloc(sizex * sizeof(int *));
    for(int i=0; i<sizex;i++)
    {
        this->pMap[i] = (int *) malloc(sizey*sizeof(int));
    }

    //pMap开始拷贝值
    for(int x=0;x<sizex;x++)
    {
        for(int y=0;y<sizey;y++)
        {
            this->pMap[x][y] = map[x][y];
        }
    }

}

Astar::~Astar()
{}

bool Astar::isInvalid(int x, int y)
{
    if (x <0 || x>=sizex || y <0 || y>=sizey)
    {
        return false;
    }
    return true;
}

void Astar::checkPoint(int x, int y, Node* father, int g)
{
    if (!isInvalid(x,y)) return;
    if (isObs(x,y)) return;
    if(isContains(&closeList,x,y) != -1) return;

    int index;
    if ((index = isContains(&openList, x, y)) != -1)
    {
        Node* point = openList[index];
        if(point->g > father->g + g)
        {
            point->father = father;
            point->g = father->g + g;
            point->f = point->g + point->h;
        }
    }
    else
    {
        Node* point = new Node(x,y,father);
        countGHF(point,endPos,g);
        openList.push_back(point);
    }
}
void Astar::NextStep(Node* currentPoint)
{
    checkPoint(currentPoint->x-1,currentPoint->y,currentPoint,10);
    checkPoint(currentPoint->x+1,currentPoint->y,currentPoint,10);
    checkPoint(currentPoint->x,currentPoint->y+1,currentPoint,10);
    checkPoint(currentPoint->x,currentPoint->y-1,currentPoint,10);

    checkPoint(currentPoint->x-1,currentPoint->y+1,currentPoint,14);
    checkPoint(currentPoint->x-1,currentPoint->y-1,currentPoint,14);
    checkPoint(currentPoint->x+1,currentPoint->y+1,currentPoint,14);
    checkPoint(currentPoint->x+1,currentPoint->y-1,currentPoint,14);
}
int Astar::isContains(vector<Node*>* Nodelist, int x, int y)
{
    for( int i = 0; i<(int)Nodelist->size();i++)
    {
        if(Nodelist->at(i)->x == x && Nodelist->at(i)->y == y)
        {
            return i;
        }
    }
    return -1;
}
void Astar::countGHF(Node* sNode, Node* eNode, int g)
{
    double h1 = abs(sNode->x - eNode->x)*10;
    double h2 = abs(sNode->y - eNode->y)*10;
    double h = h1+h2;
    //double h = min(h1,h2)*1.414 + max(h1,h2) - min(h1,h2);
    double currentg = sNode->father->g + g;
    double f = currentg + h;
    sNode->f = f;
    sNode->h = h;
    sNode->g = currentg;
}
bool Astar::compare(Node* n1, Node* n2)
{
    return n1->f < n2->f;
}
bool Astar::isObs(int x, int y)
{
    int thr = 0;
    if (x<thr || x+thr>sizex || y<thr || y+thr > sizey) return true;
    for(int dx = -thr;dx<=thr;dx++)
    {
        for(int dy=-thr;dy<=thr;dy++)
        {
            int idx = x + dx;
            int idy = y + dy;
            if(pMap[idx][idy]==1)
            {return true;}
        }
    }
    return false;
}

vector<Node*> Astar::search()
{
    cout<<"search"<<endl;
    if(isObs(startPos->x,startPos->y) || isObs(endPos->x,endPos->y)) return path;
    Node* current;
    openList.push_back(startPos);

    while (openList.size()>0)
    {
        current = openList[0];
        if (current->x == endPos->x && current->y == endPos->y)
        {
            cout<<"find the path"<<endl;
            printPath(current);
            printMap();
            break;
        }
        NextStep(current);
        closeList.push_back(current);
        openList.erase(openList.begin());
        sort(openList.begin(),openList.end(),compare);
    }
    return path;
}

void Astar::printMap()
{
    for(int i=0; i<sizex; i++)
    {
        for(int j=0;j<sizey;j++)
        {
            if(pMap[i][j]==0)
            {cout<<pMap[i][j]<<" ";}
            else if(pMap[i][j]==1)
            {cout<<"*"<<" ";}
            else
            {cout<<"+"<<" ";}
            
        }
        cout<<endl;
    }
}

void Astar::printPath(Node* current)
{
    if(current->father != nullptr)
    {
        printPath(current->father);
    }
    //cout<<"x: "<<current->x<<"y: "<<current->y<<endl;
    pMap[current->x][current->y] = 6;

    Node* point = new Node(current->x,current->y);
    path.push_back(point);
}