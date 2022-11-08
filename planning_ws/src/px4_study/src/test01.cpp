/*
头文件，包括常见的geometry_msgs和mavros通信需要的mavros_msgs，添加上就行
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;


/*
current_state表示的是无人机的状态，在主函数中订阅了对应的话题，这个状态就会不断更新，表示无人机当前的状态。state_cb就是对应的回调函数，会不断的执行，更新状态。现在先不用管为什么需要这个状态，后面的代码会解释。
*/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
	// 订阅无人机的状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    /* 
    发布一个geometry_msgs::PoseStamped的消息，需要知道的是，这个消息是控制无人机的一种方式，将指定坐标包裹进这个消息，然后发布出去，无人机就能自动飞行到指定的坐标地点
    */ 
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    
    /*
    无人机有一个锁，如果不解锁，无人机虽然接受了命令但是不会动被锁住了，只有解锁了才能对无人机进行控制，下面这个服务调用就是用来请求解锁无人机。上面的current_state就包含了无人机是否解锁的信息，若没解锁就需要解锁，否则就不用，其用途在这就体现出来
    */
   //客户端   请求解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    
    /*
    无人机飞行有很多种模式，如果需要用代码操控无人机，我们就需要切换到OFFBOARD模式。上面的current_state也包含了无人机当前的飞行模式，若不是OFFBOARD就需要切换到该模式。下面的这个服务调用就是用来请求切换无人机飞行模式。
    */
   //客户端 请求切换到offboard模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    /*
    在OFFBOARD模式下，需要以>=2HZ的频率向无人机发送消息，否则无人机会回退到OFFBOARD模式之前所在的模式，因此这里的rate需要设置的比2大就行
    */
    ros::Rate rate(20.0);
	
    // wait for FCU connection
    /*
    等待无人机与控制站连接（代码的方式就是代理），只有连接了才能发送消息
    */
    // while(ros::ok() && !current_state.connected){
    //     cout<<"no connected!"<<endl;
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    /*
    pose就是坐标，本实例是让无人机在2m处悬空，因此z设置为2，z表示的就是高度
    */
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 下面这个感觉有没有都无所谓
    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // 请求的切换模式的消息，设置为OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 请求解锁的消息，arm表示解锁，设置为true，disarm是上锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 记录上次请求的时间
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // 如果无人机模式不是OFFBOARD并且离上次操作时间大于5秒就发送请求切换，这里的5s是为了演示清楚设置的延时
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){     //请求成功
                ROS_INFO("Offboard enabled");
            }
            // 更新本次请求的时间
            last_request = ros::Time::now();
        } else {
            // 如果当前未解锁且与请求时间大于5s，就发送请求解锁
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		
        // 不断发送位置消息，但是只有解锁后才能真正开始运动，如果不发送就会退出OFFBOARD模式，因为请求发送速度要>=2HZ
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
