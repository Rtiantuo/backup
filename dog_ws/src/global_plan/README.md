# global_plan
包含了有关机器狗建图、路径规划、局部路径规划的所有的包

# Installation
1. mkdir -p ~/catkin_ws/src #新建工作空间
2. cd ~/catkin_ws/src  #进入
3. catkin_init_workspace #初始化工作空间
4. cd ..
5. catkin_make
6. cd src
7. git clone https://gitlab.itsmith.cn:7743/robot-algorithm/global_plan.git
8. cd ..
9. catkin_make #编译
10. source devel/setup.bash


# 具体内容说明
每个ros package之间都是解耦的，可以相互独立运行。后续还会对结构进行更改，对内容进行进一步的解耦和集成。最终puppy_controller将独立集成所有的功能。而其他的包也可以独立运行各自的功能，以供代码的学习和使用。

## build_map
建立占据栅格地图，及利用astar算法进行路径搜索并且跟踪
全局路径规划（利用改进A star做全局的路径搜索，t265相机做定位，d455相机做建图）
其中occ_map.cpp为主程序,Astar.cpp为改进Astar所在的全局路径规划部分。
注意在机器人上面做测试的话，要修改相应的话题名称，比如订阅的相机的话题名称，发布给机器人底盘控制节点的话题名称，要具体按照自己的设备话题名称来做修改。
Astar 的调用函数在occ_map.cpp的431行。

注意由于此文件要使用jsoncpp,所以首先要在电脑上安装jsoncpp依赖:
sudo apt-get install libjsoncpp-dev

## global_path
利用astar算法进行全局路径规划

## local_path
独立的DWA局部路径规划算法demo

## puppy_controller
集成了所有算法的总程序


