# occ_map

#### Description
全局路径规划（利用改进A star做全局的路径搜索，t265相机做定位，d455相机做建图）
其中occ_map.cpp为主程序,Astar.cpp为改进Astar所在的全局路径规划部分。
注意在机器人上面做测试的话，要修改相应的话题名称，比如订阅的相机的话题名称，发布给机器人底盘控制节点的话题名称，要具体按照自己的设备话题名称来做修改。
Astar 的调用函数在occ_map.cpp的431行。


#### Installation

1. mkdir -p ~/occ_map/src #新建工作空间
2. cd ~/occ_map/src  #进入
3. catkin_init_workspace #初始化工作空间
4. cd .. 
5. catkin_make
6. cd src
7. git clone https://gitlab.itsmith.cn:7743/robot-algorithm/global_plan.git
8. cd ..
9. catkin_make #编译
10. source devel/setup.bash
11. rosrun occ_map occ_map #(记得修改相应的话题名称，要修改的话题均在occ_map.cpp文件下，主要是机器人底盘控制的节点，在occ_map.cpp的464行，如果相机话题的名称发生改变是的话也需要修改，均在main函数里边)
12. rviz #启动rviz进行可视化 
13. 在rviz右上角选择 file->open config 选择occ_map下的rviz_config.rviz配置文件进行rviz配置
