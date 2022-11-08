#include "rrt.h"
#include "map.h"

double step =  1 ;//扩展的步长
double Thr = 1 ;

vector<rrtNode> RRT::initRRT(rrtNode start)
{
    init();//初始化地图参数
    vector<rrtNode> rrtTree;
    rrtTree.push_back(start);
    return rrtTree ;
}

rrtNode RRT::generateTempPoint()
{
    rrtNode randpoint;
    // randpoint.x = (double)(rand() % (mp.xmax*100))/100;
    // randpoint.y = (double)(rand() % (mp.ymax*100))/100;
    randpoint.x = ((double)rand()/RAND_MAX)*2*mp.xmax-mp.xmax;
    randpoint.y = ((double)rand()/RAND_MAX)*2*mp.ymax-mp.ymax;
    // cout<<"randpoint.x"<<randpoint.x<<endl;
    // cout<<"randpoint.y"<<randpoint.y<<endl;
    return randpoint;
}

int RRT::nearestT(rrtNode randpoint,vector<rrtNode> rrtTree)
{
    double mindist = 10000 ;
    int index = -1 ;
    for(int i=0; i <(int)rrtTree.size(); i++)
    {
        rrtNode temppoint = rrtTree[i] ;
        double dist = getxyzDistance(temppoint,randpoint);
        if (dist<mindist)
        {
            mindist = dist ;
            index = i ;
        }
    }
    return index;
}

double RRT::getxyzDistance(rrtNode node1,rrtNode node2)
{
    return  pow(pow((node1.x-node2.x),2) 
              + pow((node1.y-node2.y),2),0.5);
}

rrtNode RRT::Steer(rrtNode randpoint ,int index,vector<rrtNode> rrtTree)
{
    rrtNode newpoint ;
    rrtNode temppoint = rrtTree[index];
    double dist = getxyzDistance(temppoint,randpoint);
    // double step = 2;
    // newpoint.x =max (min(temppoint.x+(randpoint.x- temppoint.x)/dist*step,(double)mp.xmax-0.1),0.1);
    // newpoint.y =max (min(temppoint.y+(randpoint.y- temppoint.y)/dist*step,(double)mp.ymax-0.1),0.1);

    newpoint.x =max (min(temppoint.x+(randpoint.x- temppoint.x)/dist*step,(double)mp.xmax-0.1),-mp.xmax+0.1);
    newpoint.y =max (min(temppoint.y+(randpoint.y- temppoint.y)/dist*step,(double)mp.ymax-0.1),-mp.ymax+0.1);

    // cout<<"newpoint.x"<<newpoint.x<<endl;
    // cout<<"newpoint.y"<<newpoint.y<<endl;

    newpoint.preInd = index ;
    return newpoint ;
}

bool RRT::collision_detection(rrtNode newpoint,rrtNode startpoint)
{
    GridIndex startGrid = ConvertWorld2GridIndex (startpoint.x ,startpoint.y );
    GridIndex goalGrid = ConvertWorld2GridIndex ( newpoint.x , newpoint.y );
    vector< GridIndex > points = raycast( startGrid.x, startGrid.y, goalGrid.x , goalGrid.y);
    
    for (int i=0 ;i<points.size();i++)
    {
        GridIndex tmpIndex = points[i];
        int id = GridIndexToLinearIndex (tmpIndex);
        if (pMap[id]==100)
            return false;
        return true;
    }                   
}

bool RRT::ObstacleFree(rrtNode node)
{
    GridIndex grid = ConvertWorld2GridIndex (node.x,node.y);
    int id = GridIndexToLinearIndex (grid);
    if(pMap[id]==100)
        return false;
    return true;
}

vector<rrtNode> RRT::rrt_search(rrtNode rrt_start,rrtNode rrt_goal)
{
    //用rrt寻找路径点
    srand((unsigned)time(NULL));//这里以当前时间为种子

    vector<rrtNode> rrtTree = initRRT(rrt_start);

    for (int cycle = 0; cycle<1000; cycle++)
    {
        rrtNode randpoint = generateTempPoint();
        int index = nearestT(randpoint,rrtTree);
        rrtNode newpoint = Steer(randpoint,index,rrtTree);
        if(collision_detection(newpoint,rrtTree[index]))
        {
            rrtTree.push_back(newpoint);

            if(getxyzDistance(newpoint,rrt_goal)<Thr)
            {
                rrt_goal.preInd = rrtTree.size()-1;
                rrtTree.push_back(rrt_goal);
                
                break ;
         
            }
        }
    }
    cout<<"over"<<endl;

    // 生成的路径
    vector<rrtNode> rrtPath;
    int id = rrtTree.size()-1;
    while(true)
    {
        rrtPath.push_back(rrtTree[id]);
        id = rrtTree[id].preInd;
        if(id==-1)
            break;
    }

    return rrtPath;

}
