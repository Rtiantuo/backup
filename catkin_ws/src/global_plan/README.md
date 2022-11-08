# global_plan

全局规划算法

/* 代码整体注释在程序里面，利用A star算法规划得到经过指定点的全局路径*/

mkdir -p ~/catkin_ws/src  //新建ros工作空间
cd catkin_ws/src //进入
catkin_init_workspace  //初始化工作空间
cd ..  //退到上一级目录
catkin_make //编译
cd src 
git clone https://gitlab.itsmith.cn:7743/robot-algorithm/gbal_plan.git
cd ..
catkin_make //在工作空间下编译
source devel/setup.bash
rosrun global_path test_path  //执行可执行文件

/*其中global_plan/src下的global_path.cpp是主文件，astar.cpp是A star算法文件*/
