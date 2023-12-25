---
title: 控制实体小车cartographer建图
typora-root-url: ..\imgs
date: 2023-11-22 17:09:46
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# cartographer建图

## 跑通官方例程

### 下载官方bag

```sh
https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
```

### 运行bag

```sh
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/workspace/carto_ws/src/cartographer_paper_deutsches_museum.bag
```

### 添加map话题/map

![image-20231120172440456](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231120172440456.png)

### 保存地图

如果运行ERROR

`ERROR: Unable to load type [cartographer_ros_msgs/FinishTrajectory].
Have you typed 'make' in [cartographer_ros_msgs]?`

**进入工作空间**

```sh
source ./install_isolated/setup.bash 
```

停止地图构建

```
rosservice call /finish_trajectory 0
```

生成地图文件(后缀为`.pbstream`)

```sh
rosservice call /write_state "{filename: '${HOME}/Downloads/carto_map.pbstream'}"
```

将地图文件转换为map和yaml文件

```sh
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/carto_map -pbstream_filename=${HOME}/Downloads/carto_map.pbstream -resolution=0.05
```

map文件

<img src="https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231120173549879.png" alt="image-20231120173549879" style="zoom:50%;" />

yaml文件

```yaml
image: /home/autobot/Downloads/carto_map.pgm
resolution: 0.05
origin: [-109.515, -75.0479, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 实体小车建图

### 修改文件`revo_lds.lua`

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser",
  published_frame = "laser",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
```

`demo_revo_lds.launch`

```xml
<launch>
  <param name="/use_sim_time" value="true" />
 
  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename revo_lds.lua"
      output="screen">
    <remap from="scan" to="scan" />
  </node>
 
  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />" 
</launch>
```

### 启动雷达

```sh
 roslaunch delta_lidar delta_lidar.launch
```

### 运行与建图

```sh
roslaunch cartographer_ros demo_revo_lds.launch 
```

### 键盘控制运动

```sh
 python3 keyboard_control.py 
```

### 保存地图

参考上面

![image-20231120181556943](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231120181556943.png)

![image-20231120181919987](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231120181919987.png)

参考文章[手持雷达建立2D地图](https://blog.csdn.net/Fang_cheng_/article/details/111593210)

## 离线建图

### 启动雷达

```sh
 roslaunch delta_lidar delta_lidar.launch
```

### 键盘控制

```sh
 python3 keyboard_control.py 
```

### rosbag记录

```sh
mkdir ./workspace/bagfiles
```

```sh
rosbag record -a -O 目标文件
```

默认目标文件名称为`时间戳.bag`

### 启动cartographer

```sh
roslaunch cartographer_ros demo_revo_lds.launch 
```

### rosbag播放

```sh
rosbag play --clock 2023-11-21-17-45-52.bag 
```

如图所示

![image-20231122162717535](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231122162717535.png)

### 保存地图

参考上面

![image-20231122163739694](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231122163739694.png)
