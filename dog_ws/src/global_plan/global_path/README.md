# 内容说明
## test_path
用途：读取已经保存的地图。接收全部的路径点，然后规划出一条经过所有路径点的全局路径

地图来源：test_global_path第30行
路径点来源：test_global_path第140行pose_sub=nh.subscribe<nav_msgs::Path>("/plan_points",10,doPose);
全局路径点发布：test_global_path第142行