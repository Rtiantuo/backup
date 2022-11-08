#ifndef _MAP_OPERATE_H_
#define _MAP_OPERATE_H_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <iostream>
#include <math.h>


typedef struct grindindex_
{
    int x ;
    int y ;
    // int z ;

    void SetIndex (int x_ , int y_ )
    {
        x = x_ ;
        y = y_ ;
        // z = z_ ;
    }
} GridIndex ;

typedef struct  map_params
{
    double  log_occ , log_free ;
    double  resolution ; 
    double  origin_x , origin_y ;
    int sizex , sizey ;
    int xmax , ymax ;
    int offset_x , offset_y ;

} MapParams ;
MapParams mp ;

class Map_Operate{
    public:
        init();//地图初始化
        GridIndex ConvertWorld2GridIndex ( double x , double y );//从世界坐标系转换到栅格坐标系
        Vector2d ConvertGridindex2World ( int x , int y ); //栅格坐标系转换到世界坐标系
        int GridIndexToLinearIndex ( GridIndex index );//将二维的点变成1维的
        Vector3d indexToPos ( int index ); //将栅格中一维的点换成二维的
        bool isValidGridIndex ( GridIndex index ) //判断该点是否是有效点 在栅格地图以内
        vector<GridIndex> raycast ( int x1 , int y1 , int x2 , int y2 );//划线算法


};

#endif