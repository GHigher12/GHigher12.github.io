---
title:
  - ROS2显示urdf模型中No transform from [XX_Link] to [base_link]
date: 2023-03-31 22:43:42
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# ROS2显示urdf模型中No transform from [XX_Link] to [base_link]

## 错误

![image-20230331221601858](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b9c9a6b40ba61c7e97d14a5b156eabb3.png)

该错误中关节`joint`为`fixed`可以正常显示，而其他`joint`为`cotinuous`,都显示无法关联到`base_link`，当我尝试把所有joint改为fixed时可以显示正常，但这并不是正确的解决办法。

## 解决

原因是我在拉launch.py文件中未添加`joint_state_publisher`节点,导致旋转关节无法关联

`display_launch.py`

```py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    p_value = ParameterValue(Command(["xacro ", get_package_share_directory("cpp06_urdf") + "/urdf/urdf/ros_car.urdf"]))
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":p_value}]
    )
    rviz2 = Node(
        package="rviz2", 
        executable="rviz2",
        arguments=["-d", get_package_share_directory("cpp06_urdf")+"/rviz/ros_car.urdf.rviz"])
    return LaunchDescription([model, robot_state_pub, rviz2])
```

添加`joint_state_publisher`节点

```python
    joint_state_pub = Node(
        package="joint_state_publisher", 
        executable="joint_state_publisher")
    
    return LaunchDescription([model, robot_state_pub,joint_state_pub, rviz2])
```
重新编译并运行
```shell
colcon build --packages-select cpp06_urdf
. install/setup.bash
ros2 launch cpp06_urdf display_launch.py 
```

显示正常

![image-20230331222508142](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/2aacc9744ea6567c867d66202ccdbda7.png)
