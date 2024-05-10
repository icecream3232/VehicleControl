# sandtable_vehicle_ros

#### 介绍
沙盘智能车辆车载程序，基于ROS搭建，包括感知、控制、决策规划三个部分


#### 使用说明

1.  把src/目录下的功能包替换到自己的工作空间下，除了自己的功能包外，同时，也需要把common_msgs功能包和launch文件夹放到自己工作空间的src/目录下。
2.  如果测试需要底层驱动电机，则先获取root权限，在执行配置环境变量等操作。

```
sudo su
```

3.  配置ROS环境变量

```
. /opt/ros/noetic/setup.bash
```

4.  编译
5.  配置工程环境变量
```
. devel/setup.bash
```
6.  若要运行整个系统，运行launch文件
```
roslaunch src/launch/sandtable_vehicle.launch
```

7.  文件src/common_map.h是最新版沙盘地图，各功能包若使用地图，请将最新版的地图文件拷贝到功能包内
拷贝位置：功能包名/include/功能包名/
举例：v2i/include/v2i

