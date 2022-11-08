#include  "map.h"

ros::Publisher map_pub_;

//像素坐标和深度——相机坐标系下的坐标——世界坐标系
inline Eigen :: Vector3d cal_point ( int u , int v , double depth )
{
    Eigen :: Vector3d proj_pt ;
    proj_pt ( 0 ) = depth ; //x方向表示深度
    proj_pt ( 1 ) = -1 * ( u - mp . cx ) / mp . fx * depth ; //y
    proj_pt ( 2 ) = -1 * ( v - mp . cy ) / mp . fy * depth ; // z

    proj_pt = rob . Rotation * proj_pt + rob . Pose ;
    return proj_pt ;
}

//发布地图
void publishMap(const ros::TimerEvent &event )
{
    pcl :: PointXYZ pt;//
    pcl :: PointCloud <pcl::PointXYZ> cloud;//建立一个pcld的点云，不能直接发布
    for (int i=0; i<mp.sizex* mp.sizey* mp.sizez; i++)
    {
        if (pMap[i] >75)
        {
            Eigen :: Vector3d pos ;
            pos = indexToPos(i) ;//一维转成三维
            //高度大于某个值就不发布l 
            if (pos(2)>4.0) continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            // cout<<"x"<<pt.x<<" y"<<pt.y<<" z"<<pt.z<<endl;
            if (pt.z >1.5 || pt.z <-0.2) continue;
            //不跟上边那个合并一起continue吗？？？？？
            cloud.push_back(pt);
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;//有结构、无结构点云
    cloud.is_dense = true;//数据是否有限
    cloud.points.resize(cloud.width*cloud.height);

    cloud.header.frame_id = "t265_odom_frame";

    sensor_msgs :: PointCloud2 cloud_msg;//建立一个可以直接发布的点云

    pcl::toROSMsg(cloud,cloud_msg);//将点云转化为消息
    map_pub_.publish(cloud_msg);//发布出去
    cout<<"cloud.width-----------------------------------------"<<cloud.width<<endl;
    //cout<<"hello"<<endl;
}

void projectDepth ()
{
    // ros :: Time t1 , t2 ;//定义两个时间
    // t1 = ros::Time::now();//获取当前时间
    if ( ! rob . has_image ) return ;

    for ( int u = 0 ; u < 640 ; u = u + 3 )
    {
        for ( int v = 0 ; v < 480 ; v = v + 3 )
        {

            double depth = ( double ) ( rob . depth_image_.at<uint16_t>(v,u))/1000.0 ;
            //单位转换？？米
            Eigen :: Vector3d pos = cal_point ( u ,v ,depth ) ;
            // cout<<"depth----------"<<depth<<endl;
            if (depth < 0.3 || depth > 4.0) continue ;//深度太大或者 太小这个点作废了 下一个

            //起始点 ———相机当前的位置 
            double sx = rob.Pose(0) , sy = rob.Pose(1) , sz = rob.Pose(2) ;
            GridIndex start_index = ConvertWorld2GridIndex(sx , sy ,sz);
            //终点 ————uv
            double gx = pos(0) , gy = pos(1) ,gz = pos(2) ;
            GridIndex goal_index = ConvertWorld2GridIndex(gx , gy , gz);
            //障碍物的位置(栅格的形式)
            GridIndex occ_index;
            occ_index.SetIndex(goal_index.x, goal_index.y, goal_index.z);
            //障碍物的位置超出了栅格就不考虑了
            if (!isValidGridIndex(occ_index)) continue;

            int occ_idx = GridIndexToLinearIndex(occ_index);//障碍物三维变一维
            pMap[occ_idx] += mp.log_occ ; 
            if (pMap[occ_idx] >= 100) pMap[occ_idx] = 100;

            auto points = raycast(start_index.x, start_index.y, start_index.z,
                                  goal_index.x, goal_index.y, goal_index.z);

            for (int i = 0; i<(int) points.size(); i++)
            {
                GridIndex free_index;
                free_index.SetIndex(points[i].x, points[i].y, points[i].z);
                int idx = GridIndexToLinearIndex(free_index);
                pMap[idx] += mp.log_free ;
                //
                if (pMap[idx] <= 10)  pMap[idx] = 10;
            }
        }
    }
   // t2 = ros::Time::now();//获取当前时间
}

//按顺序？？？
void updateMap ( const ros :: TimerEvent & event )
{
    //处理深度图
    projectDepth () ;
}

//回调
void depthOdomCallback ( const nav_msgs:: OdometryConstPtr &odom , const sensor_msgs::ImageConstPtr &img )
{
   // cout <<"callback1----------------"<<endl;
    //获取位姿 旋转 矩阵
    rob . Pose ( 0 ) = odom -> pose.pose.position.x;
    rob . Pose ( 1 ) = odom -> pose.pose.position.y;
    rob . Pose ( 2 ) = odom -> pose.pose.position.z;

    rob.Rotation = Eigen::Quaterniond (odom -> pose.pose.orientation.w ,
                                       odom -> pose.pose.orientation.x ,
                                       odom -> pose.pose.orientation.y ,
                                       odom -> pose.pose.orientation.z ).toRotationMatrix();

    double roll , pitch , yaw ;
    tf :: Quaternion q ;
    tf :: quaternionMsgToTF ( odom -> pose.pose.orientation , q );
    tf :: Matrix3x3 ( q ) . getRPY ( roll , pitch , yaw );
    rob . theta =  yaw;

    //解码相机消息
    cv_bridge :: CvImagePtr cv_ptr;
    cv_ptr = cv_bridge :: toCvCopy (img , img -> encoding);
    rob . depth_image_ = cv_ptr -> image;
    if (rob . has_image == false)
        rob . has_image = true;
    return ; 
}

int main (int argc , char  ** argv )
{
    ros :: init ( argc , argv , "map_build" ) ;
    ros :: NodeHandle n ;
    cout <<"开始"<<endl;
    init();

    //同步订阅
    Subscriber <nav_msgs::Odometry> odom_sub ( n, "/t265/odom/sample" ,  10 ) ;
    Subscriber <sensor_msgs::Image> depth_sub ( n, "/d400/depth/image_rect_raw" ,  10 ) ;
   
    typedef sync_policies::ApproximateTime <nav_msgs::Odometry ,  sensor_msgs::Image>  MySyncPolicy ;
    Synchronizer <MySyncPolicy> sync(MySyncPolicy ( 10 ) ,  odom_sub ,  depth_sub ) ;
    sync.registerCallback ( boost::bind ( &depthOdomCallback ,  _1 ,  _2 ) ) ;
    
    //f发布消息
    map_pub_  = n.advertise<sensor_msgs::PointCloud2>("/my_map/occupancy",10);
    //计时器—更新占用地图
    ros :: Timer update_timer_ = n . createTimer ( ros :: Duration ( 0.035 ) , updateMap) ;
    // cout <<"更新"<<endl;
    //计时器—发布地图
    ros :: Timer publisher_map_timer_ = n . createTimer ( ros :: Duration (0.2) , publishMap ) ;

    // cout <<"发布"<<endl;
    ros::spin();
    return 0;

}
