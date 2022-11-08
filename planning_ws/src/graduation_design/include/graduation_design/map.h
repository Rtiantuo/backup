#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/StdVector>

using namespace std;
//静止地图和动态地图
int *pMap;

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

//地图初始化
void init()
{
    mp . sizex = 200 ;
    mp . sizey = 200 ;
    mp . resolution = 0.1 ;

    mp . origin_x = 0 ;
    mp . origin_y = 0 ;

    mp . offset_x = mp.sizex/2.0 ;
    mp . offset_y = mp.sizey/2.0 ;

    mp . xmax = (mp . sizex - mp . offset_x)* mp . resolution;
    mp . ymax = (mp . sizey - mp . offset_y)* mp . resolution ;
    pMap = new int [ mp . sizex * mp . sizey ] ; 
     //初始化这个
    for (int i=0 ; i < mp . sizex * mp . sizey ; i++)
        pMap [ i ] = 50 ;
}

//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex ( double x , double y )
{
    GridIndex index ; 
    //正负10米？
    index . x =  round ( ( x - mp . origin_x ) / mp . resolution ) + mp . offset_x ;
    index . y =  round ( ( y - mp . origin_y ) / mp . resolution ) + mp . offset_y ;
    return index ;
}

//栅格坐标系转换到世界坐标系
Eigen :: Vector2d ConvertGridindex2World ( int x , int y )
{
    double px = ( x - mp.offset_x ) * mp.resolution + mp.origin_x ;
    double py = ( y - mp.offset_y ) * mp.resolution + mp.origin_y ;

    return Eigen :: Vector2d ( px , py );
}

//将二维的点变成1维的
int GridIndexToLinearIndex ( GridIndex index )
{
    int linear_index ;
    linear_index = index . y * mp . sizex + index . x ;
    return linear_index ;
}

//将栅格中一维的点换成二维的
Eigen :: Vector3d indexToPos ( int index )
{
    Eigen :: Vector3d pos ; 
    int idy = index / mp . sizex ;
    int idx = index % mp . sizex ;

    double px = ( idx - mp.offset_x ) * mp.resolution + mp.origin_x ;
    double py = ( idy - mp.offset_y ) * mp.resolution + mp.origin_y ;
    double pz = 0;
    
    return Eigen :: Vector3d ( px , py ,pz);
}

//判断该点是否是有效点 在栅格地图以内
bool isValidGridIndex ( GridIndex index )
{
    if ( index.x >= 0 && index.x < mp.sizex 
    && index.y >= 0 && index.y < mp.sizey )
        return true ;
    
    return false ;
}

vector<GridIndex> raycast ( int x1 , int y1 , int x2 , int y2 )
{
    GridIndex tmpIndex ;
    vector < GridIndex > gridIndexVector ;
    int dx , dy ;
    int sx , sy ;

    dx = abs ( x1 - x2 ) ;
    dy = abs ( y1 - y2 ) ;
    if ( x1 > x2 ) sx = -1 ; else sx = 1 ;
    if ( y1 > y2 ) sy = -1 ; else sy = 1 ;

    int PX , PY ;
    if ( dx >= dy)
    {
        int y = y1 ;
        int errorY = 0 ;
        for (int x = x1 ; x != x2 ; x = x + sx)
        {
            PX = x ;
            PY = y ;

            errorY += dy ; 

            if ( 2 * errorY > dx )
            {   
                y += sy ;
                errorY -= dx ;
            }
            
            if (PX == x2 && PY == y2) continue ;//???
            //  gridIndex里面不包含(x2,y2,z2)和(x1,y1,z1)吧？
            tmpIndex.SetIndex ( PX , PY) ;
            gridIndexVector.push_back ( tmpIndex ) ;
         }
    }

    else if ( dy >= dx)
    {
        int x = x1 ;
        int errorX = 0 ;
       
        for (int y = y1 ; y != y2 ; y = y + sy)
        {
            PX = x ;
            PY = y ;
            errorX += dx ; 
            if ( 2 * errorX > dy )
            {   
                x += sx ;
                errorX -= dy ;
            }

            if (PX == x2 && PY == y2) continue ;//???
            //  gridIndex里面不包含(x2,y2,z2)和(x1,y1,z1)吧？
            tmpIndex.SetIndex ( PX , PY) ;
            gridIndexVector.push_back ( tmpIndex ) ;
         }
    }
    return gridIndexVector;
}


#endif
