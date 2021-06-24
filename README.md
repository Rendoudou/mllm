Multi level lidar Map
===================================

## 项目目的
    实现基于2-D激光雷达的多级地图定位

## 依赖项目
    Ubuntu 16.04
    ROS-Kinetic
    ros-package: laser_filters
    ros-package: scan_tools: laser_scan_matcher
    ros-package: scan_tools: scan_to_cloud_converter
    ros_package: slam_gmapping
    ros-package: hector_slam
    ros-package: navigation
    接收的点云话题名为 /scan
    激光雷达 HOKUYO 10XL

## 项目进展记录

#### date    20210602
    #version v0.1
    #statue  processing
    #brief   1.录制了Hokuyo激光雷达采集的点云数据包。
             2.使用laser_filters对laserScan进行滤波，去除拖延点，插值滤波，截取-130度至130度。
             3.使用laser_scan_matcher获取关键帧位姿（具体关键帧规则还未确定）。
             4.对laser_scan_matcher.cpp 做小修改, 以RJY做标记。
             5.对scan_to_cloud_converter.cpp 做小修改, 以RJY做标记。
             6.tf树为 map->odom->base_link->laser_frame。
             7.createNode 接受点云和位姿，都为关键节点的位姿，且实现了同步接收。

#### date    20210603
    #version v0.2
    #statue  processing
    #brief   1.设置节点规则。node_liner_limit、node_angular_limit。
             2.记录节点时间戳和节点信息。save_path。
             3.节点角度需求判断用tf2::getYaw效果好于Eigen::eulerAngles。

#### date    20210607
    #version v0.3
    #statue  processing
    #brief   1.保存节点的orb、点云与图片，saveORBPcd。
             2.保存节点的Pose，savePose。
             3.生成点云地图，转化为栅格地图，createGridMap。
             4.生成的点云地图效果不好，考虑用gmapping代替。

#### date    20210608
    #version v0.4
    #statue  processing
    #brief   1.使用gmapping生成栅格地图，slam gammping。
             2.为了解决tf树的问题，将scan_filtered之后的点云时间戳都设置为ros::Time::now()
               最初的scan话题不能显示，由于时间戳的问题。
             3.加入霍夫变换求取点云中的直线，但是所求直线难以计算真值。

#### date    20210609
    #version v0.5
    #statue  processing
    #brief   1.给节点增加红色的球状标记。
             2.添加.gitignore。
             3.将scan_tools的分支切换为hydro, 切换为hydro分支后编译卡顿。
	           由于indigo分支对于实际运行无影响，所以换回indigo分支。
             4.节点的真实值问题还是有点麻烦。

#### date    20210621
    #version v0.6
    #status  processing
    #brief   1.修改launch文件，construct_map_gmapping.launch、construct_map_hector.launch。
             2.新增hector_slam功能包，对比gmapping生成的栅格地图。
             3.使用navigation/map_server/map_saver保存栅格地图。
             4.对比生成的两种栅格地图，hector_slam效果略优。
             5.使用map_saver保存生成的栅格地图
             6.下一步方向，用hector_slam的里程计结果作为节点位姿的真值，做图优化。

#### date    20210624
    #version v0.7
    #brief   1.特征提取修改为全局orb,修正了yaml的保存问题，前一版本保存格式不利于查找。
             2.实现了相似节点的快速查找，即节点级定位，采用暴力匹配的方案。暴力匹配不好，考虑wifi定位做粗定位。
             3.增加的内容主要在localization.cpp中，add localization.launch、view_localization.rviz
             4.当前粗略的实现了输入点云的最优节点查找，下一步要实现度量级别定位。
             5.实际运行下来发现开源的hector_slam与gmapping的里程计结果与scan_matcher的结果非常接近。还有必要做g2o优化里程计吗？
             6.下一步目标实现度量级别定位并且输出预测的位姿，transformLaserScanToPointCloud api很好用。
             7.可视化部分加入了预先构建的地图与存储的节点，下一步将加入预测的位姿。
             8.添加navigation包只用做加载地图与保存地图使用。
             

## 使用方法
#### 使用仓库提供的小例子创建未完善的地图与定位
    ### construct map ###
    # new bash
    # mkdir catkin_ws_mllm
    # cd catkin_ws_mllm
    # mkdir src
    # cd src
    # git clone https://github.com/Rendoudou/mllm.git
    # cd ..
    # catkin_make
    # source devel/setup.bash
    # roslaunch multi_level_lidar_map construct_map_hector.launch
    # ...
    # new bash
    # rosrun map_server map_saver -f hector_map map:=/hector_map

    ### localization ###
    # new bash
    # cd catkin_ws_mllm
    # source devel/setup.bash
    # roslaunch multi_level_lidar_map localization.launch
