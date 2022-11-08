#include"puppy_controller/dwa_local_planner.h"
bool flag=false;
//定义路径点
vector<Point*> Path;

//订阅全局path
void path_Callback(const nav_msgs::PathConstPtr& path)
{
    Path.clear();
    for(int i = 0;i<path->poses.size();++i)
    {
        double x = path->poses[i].pose.position.x;
        double y = path->poses[i].pose.position.y;
        Point* pt=new Point(x,y);
        Path.push_back(pt);
    }
    cout<<"path.size: "<<Path.size()<<endl;
    if(Path.size() != 0)
    flag =true;
}
//获取当前点的局部目标点
int near_pt(double px,double py)
{
    double d=1000;
    int idx=0;
    for(int i=0;i<(int)Path.size();++i)
    {
        double x=Path[i]->x;
        double y=Path[i]->y;
        double dist=pow((px-x)*(px-x)+(py-y)*(py-y),0.5);
        if(dist<d)
        {
            d=dist;
            idx=i;
        }
    }
    if(idx >= Path.size()-3)
    {
        return Path.size()-1;
    }
    return idx+2;
}

//底盘速度订阅回调函数
void vel_Callback(const nav_msgs::OdometryConstPtr& vel)
{
    //cout<<"int the vel callback!"<<endl;
    vx=vel->twist.twist.linear.x;
    wz=vel->twist.twist.angular.z;
}
//t265回调函数  改变当前位置 获得yaw  得到局部路径的目标点
void odom_Callback(const nav_msgs::OdometryConstPtr& odom)
{
    //cout<<"in the odom callback!"<<endl;
    //获取当前位置点和yaw角
    robot_state.position(0)=odom->pose.pose.position.x;
    robot_state.position(1)=odom->pose.pose.position.y;
    robot_state.position(2)=odom->pose.pose.position.z;
    double roll,pitch,yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation,q);
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
    robot_state.yaw=yaw;
    //获取当前局部路径的目标点
    if(flag == false)  return;
   int idx=near_pt( robot_state.position(0), robot_state.position(1));
   robot_state.goal_now<<Path[idx]->x,Path[idx]->y;
//    cout<<"near pt:"<<Path[idx]->x<<" "<<Path[idx]->y<<endl;
}

//计算plan_t后的状态
void  cal_state(RobotState& state)
{
    state.position(0)=state.position(0)+dwa_param.dt*cos(state.yaw)*free_vx;
    state.position(1)=state.position(1)+dwa_param.dt*sin(state.yaw)*free_vx;
    state.yaw=state.yaw+dwa_param.dt*free_wz;
}

void dwa_plan(const ros::TimerEvent& event)
{
    if(flag == false)  return;
    clock_t time_stt=clock();  //计时
    //step1.计算当前状态下的可行速度
    double v_min=max(-robot_param.v_max,vx-robot_param.v_acc_max*dwa_param.dt);
    double v_max=min(robot_param.v_max,vx+robot_param.v_acc_max*dwa_param.dt);
    double w_min=max(-robot_param.w_max,wz-robot_param.w_acc_max*dwa_param.dt);
    double w_max=min(robot_param.w_max,wz+robot_param.w_acc_max*dwa_param.dt);
    //step2.进行速度采样并打分
    double score=-100000;
    for(int i=0;i<=dwa_param.smaple_num(0);++i)   //速度采样空间
    {
        for(int j=0;j<=dwa_param.smaple_num(1);++j)  //角速度采样空间
        {
            //每个速度采样
            free_vx=v_min+(i/dwa_param.smaple_num(0))*(v_max-v_min);
            free_wz=w_min+(j/dwa_param.smaple_num(1))*(w_max-w_min);
            //计算经过plan_t时间后的状态
            double time=0;
            RobotState state;
            state.position(0)=robot_state.position(0);
            state.position(1)=robot_state.position(1);
            state.position(2)=robot_state.position(2);
            state.yaw=robot_state.yaw;
            while(time<=dwa_param.plan_t)
            {
                time+=dwa_param.dt;
                cal_state(state);
            }
            //进行评分
            //--1.heading
            double goal_theta=atan2(robot_state.goal_now(1)-state.position(1),robot_state.goal_now(0)-state.position(0));
            double heading=goal_theta-state.yaw;
            if(heading>3.1415)  heading -=3.1415*2;
            if(heading<-3.1415) heading+=3.1415*2;
            heading=fabs(180-toDegree(fabs(heading)));
            //--2.最大速度评分
            double vel=fabs(free_vx);
            //--3.距离局部目标点的距离评分
            double dist=sqrt((state.position(0)-robot_state.goal_now(0))*(state.position(0)-robot_state.goal_now(0))+
                                                (state.position(1)-robot_state.goal_now(1))*(state.position(1)-robot_state.goal_now(1)));
           double state_score=heading*dwa_param.evalCB(0)+vel*dwa_param.evalCB(1)-dist*dwa_param.evalCB(2);
           //double state_score=vel*dwa_param.evalCB(1)-dist*dwa_param.evalCB(2);
            //--计算最高得分状态控制
            if(state_score>score)
            {
                score=state_score;
                cal_vx=free_vx;
                cal_wz=free_wz;
                toX=state.position(0);
                toY=state.position(1);
            }
            //cout<<"============"<<endl;
        }
    }
    //cout<<"====time used:"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<" ms"<<endl;
}

double toDegree(double theta)
{
    double degree=theta/3.1415*180;
    return degree;
}

void publish_vel(const ros::TimerEvent& event)
{
    if(flag == false)  return;
    //cout<<"in the vel pub!"<<endl;
    cout<<"cal_vel:"<<cal_vx<<" wz "<<cal_wz<<endl;
    //cout<<"toXY:"<<toX<<"  "<<toY<<endl;
    geometry_msgs::Twist twist;
    double goal_x=Path[(int)Path.size()-1]->x;
    double goal_y=Path[(int)Path.size()-1]->y;
    double dist2goal=sqrt((goal_x-robot_state.position(0))*(goal_x-robot_state.position(0))+
                                                    (goal_y-robot_state.position(1))* (goal_y-robot_state.position(1)));
    if(dist2goal<0.15)
        {
            cout<<"====arrive gola==="<<endl;
            cal_vx=0.0;
            cal_wz=0.0;
        }
    twist.linear.x=cal_vx;
    twist.angular.z=cal_wz;
    vel_pub.publish(twist);
}


int main(int argc, char *argv[])
{
    cout<<"hello!"<<endl;
    init();
    ros::init(argc,argv,"dwa");
    ros::NodeHandle nh("~");
    //订阅全局路径
    ros::Subscriber path_sub=nh.subscribe<nav_msgs::Path>("/my_map/global_path",10,path_Callback);
    //订阅turtlebot底盘速度
    vel_sub=nh.subscribe<nav_msgs::Odometry>("/odom",10,vel_Callback);
    //t265订阅回调函数
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample",10,odom_Callback);
    //进行dwa规划
    ros::Timer dwa_timer=nh.createTimer(ros::Duration(0.1),dwa_plan);
    //发布机器人控制
    vel_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",10);
    ros::Timer vel_timer=nh.createTimer(ros::Duration(0.05),publish_vel);

    ros::spin();
    return 0;
}
