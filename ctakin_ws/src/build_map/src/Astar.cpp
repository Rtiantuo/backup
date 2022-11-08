#include"build_map/Astar.h"

//地图初始化
Astar::Astar(Node* startPos,Node* endPos,int* map,int sizex,int sizey,int sizez)
{
    this->startPos=startPos;
    this->endPos=endPos;
    this->sizex=sizex;
    this->sizey=sizey;
    this->sizez=sizez;

    //二维数组分配内存
    pMap=(int**)malloc(sizex*sizeof(int*));
    for(int i=0;i<sizex;++i)
    {
        pMap[i]=(int*)malloc(sizey*sizeof(int));
    }

     int temp_map[sizex][sizey];

    for(int x=0;x<sizex;x++)
    {
        for(int y=0;y<sizey;y++)
        {
            temp_map[x][y] = 0;
            for(int z=10;z<=30;z++)   
            {
                int index = z*sizex*sizey + y*sizex + x;
                if(map[index] > 70)
                {
                    temp_map[x][y] += 3;
                }
                if(map[index] < 20)
                {
                    temp_map[x][y] -= 1;
                }
            }
        }
    }

        for(int x=0; x<sizex;x++)
        {
        for(int y=0;y<sizey;y++)
        {
            this->pMap[x][y] = 0;   //没障碍物的地方设置为0  
            if (temp_map[x][y]>0)
            {
                for(int i=-3;i<=3;i++)
                {
                    for(int j=-3;j<=3;j++)
                    {
                        int tx = x+i;
                        int ty = y+j;
                        if(tx <0 || tx >=sizex || ty <0 || ty >=sizey) continue;
                        this->pMap[tx][ty]=1;    //--有障碍物的地方设置为1  膨胀三层
                    }
                }
            }
        }
    }

}

int Astar::gridindex2Lineindex(int x,int y)
{
    int LineIndex=y*100+x;
    return LineIndex;
}

vector<Node*> Astar::search()
{
    cout<<"start search!"<<endl;
    if(isObs(startPos->x,startPos->y) || isObs(endPos->x,endPos->y)) return path;
    Node* current;
    openList.push_back(startPos);
    while (openList.size()>0)
    {
        current=openList[0];
        float dx=fabs(current->x-endPos->x);
        float dy=fabs(current->y-endPos->y);
        if((dx*dx+dy*dy)<3)
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
    checkPoint(currentPoint->x+step,currentPoint->y,currentPoint);
    checkPoint(currentPoint->x-step,currentPoint->y,currentPoint);
    checkPoint(currentPoint->x,currentPoint->y+step,currentPoint);
    checkPoint(currentPoint->x,currentPoint->y-step,currentPoint);

    checkPoint(currentPoint->x-sin30Len,currentPoint->y-cos30Len,currentPoint);
    checkPoint(currentPoint->x-sin30Len,currentPoint->y+cos30Len,currentPoint);
    checkPoint(currentPoint->x+sin30Len,currentPoint->y-cos30Len,currentPoint);
    checkPoint(currentPoint->x+sin30Len,currentPoint->y+cos30Len,currentPoint);

    checkPoint(currentPoint->x-cos30Len,currentPoint->y-sin30Len,currentPoint);
    checkPoint(currentPoint->x-cos30Len,currentPoint->y+sin30Len,currentPoint);
    checkPoint(currentPoint->x+cos30Len,currentPoint->y-sin30Len,currentPoint);
    checkPoint(currentPoint->x+cos30Len,currentPoint->y+sin30Len,currentPoint);
}

void Astar::checkPoint(float x,float y,Node* father)
{
   int zx=round(x);
   int zy=round(y);
    if(isInvalid(zx,zy))  return;
    if(isObs(zx,zy))  return;
    if(isContains(&closeList,zx,zy)!=-1) return;

     int index;
     if((index=isContains(&openList,zx,zy))!=-1)
     {
         Node* point=openList[index];
         if(point->g>(father->g+step))
         {
             point->father=father;
             point->g=father->g+step;
             point->f=point->g+point->h;
             point->x=x;
             point->y=y;
         }
     }
     else{
         Node* point=new Node(x,y,father);
         countGHF(point,endPos);
         openList.push_back(point);
     }
}

void Astar::countGHF(Node* sNode,Node* eNode)
{
    int h1=abs(sNode->x-eNode->x);
    int h2=abs(sNode->y-eNode->y);
    float h=h1+h2;
    float currentG=sNode->father->g+step;
    float f=currentG+h;
    sNode->f=f;
    sNode->h=h;
    sNode->g=currentG;
}

int Astar::isContains(vector<Node*>* Nodelist,int x,int y)
{
    for(int i=0;i<(int)Nodelist->size();++i)
    {
        if(Nodelist->at(i)->x==x && Nodelist->at(i)->y==y)
        {
            return i;
        }
    }
    return -1;
}

bool Astar::isInvalid(int x,int y)
{
    if(x<0 || x>sizex || y<0 || y>sizey )
    {
        return true;
    }
    return false;
}

bool Astar::compare(Node* n1,Node* n2)
{
    return n1->f<n2->f;
}

bool Astar::isObs(int x,int y)
{
    if(pMap[x][y]==1)
    {
        return true;
    }
    return false;
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
    path.push_back(endPos);
}