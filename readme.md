先把点云可视化出来  
map->camera_init  
camera_init->livox  

角速度环  
角度环（Point-LIO）输出姿态  
水平速度环

位置环  
导航  
识别

遥控器


遥控器  
1. 横滚
2. 俯仰
3. 油门
4. 偏航

RosParam:是否遥控
(default):0 也能用遥控拨杆设成0
1. off-board
2. position
3. stable
4. acro

遥控程序启动后修改param
其他程序作出反映

TODO:
    lidar imu 標定
    imu 方差測量
