#include "map_operate.h"

//地图初始化
void init()
{
    mp . sizex = 300 ;
    mp . sizey = 300 ;
    mp . resolution = 0.1 ;

    mp . origin_x = 0 ;
    mp . origin_y = 0 ;

    mp . offset_x = 0 ;
    mp . offset_y = 0 ;

    mp . xmax = 30 ;
    mp . ymax = 30 ;
    // pMap = new int [ mp . sizex * mp . sizey ] ; 
    //  //初始化这个
    // for (int i=0 ; i < mp . sizex * mp . sizey ; i++)
    //     pMap [ i ] = 50 ;
}

//从世界坐标系转换到栅格坐标系
GridIndex Map_Operate::ConvertWorld2GridIndex ( double x , double y )
{
    GridIndex index ; 
    //正负10米？
    index . x =  round ( ( x - mp . origin_x ) / mp . resolution ) + mp . offset_x ;
    index . y =  round ( ( y - mp . origin_y ) / mp . resolution ) + mp . offset_y ;
    return index ;
}

//栅格坐标系转换到世界坐标系
Vector2d Map_Operate::ConvertGridindex2World ( int x , int y )
{
    double px = ( x - mp.offset_x ) * mp.resolution + mp.origin_x ;
    double py = ( y - mp.offset_y ) * mp.resolution + mp.origin_y ;

    return Eigen :: Vector2d ( px , py );
}

//将二维的点变成1维的
int Map_Operate::GridIndexToLinearIndex ( GridIndex index )
{
    int linear_index ;
    linear_index = index . y * mp . sizex + index . x ;
    return linear_index ;
}

//将栅格中一维的点换成二维的
Vector3d Map_Operate::indexToPos ( int index )
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
bool Map_Operate::isValidGridIndex ( GridIndex index )
{
    if ( index.x >= 0 && index.x < mp.sizex 
    && index.y >= 0 && index.y < mp.sizey )
        return true ;
    
    return false ;
}

vector<GridIndex> Map_Operate::raycast ( int x1 , int y1 , int x2 , int y2 )
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