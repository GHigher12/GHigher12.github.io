---
title: ROS-Moveit机械臂追踪二维码（四）
typora-root-url: ..\imgs
date: 2023-07-22 18:31:17
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# ROS-Moveit机械臂追踪二维码(四)

## 在仿真环境增加相机

```xml
<gazebo reference="camera_depth_frame">
    <sensor name="camera1" type="depth">
      <always_on>true</always_on>
      <update_rate>20.0</update_rate>
      <camera>
        <horizontal_fov>1.0471975511965976</horizontal_fov>
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.05</near>
          <far>8.0</far>
        </clip>
      </camera>
      <plugin filename="libgazebo_ros_openni_kinect.so" name="camera_depth_frame_kinect_controller">
        <baseline>0.1</baseline>
        <alwaysOn>true</alwaysOn>
        <updateRate>10</updateRate>
        <cameraName>camera1</cameraName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <depthImageTopicName>depth/image_raw</depthImageTopicName>
        <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
        <pointCloudTopicName>depth/points</pointCloudTopicName>
        <frameName>camera_depth_optical_frame</frameName>
        <pointCloudCutoff>0.3</pointCloudCutoff>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
        <!--
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
      -->
      </plugin>
    </sensor>
  </gazebo>
  <joint name="camera_rgb_joint" type="fixed">
        <!-- 以下rpy xyz参数为相机位置可适当修改-->
    <origin rpy="3.142 1.5706 3.142" xyz="0.35 0 1"/>
    <parent link="world"/>
    <child link="camera_rgb_frame"/>
  </joint>
  <link name="camera_rgb_frame">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="camera_rgb_optical_joint" type="fixed">
    <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
    <parent link="camera_rgb_frame"/>
    <child link="camera_rgb_optical_frame"/>
  </joint>
  <link name="camera_rgb_optical_frame">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <origin rpy="0 0 0" xyz="-0.031 0 -0.016"/>
    <parent link="camera_rgb_frame"/>
    <child link="camera_link"/>
  </joint>
  <link name="camera_link">
    <visual>
      <origin rpy="0 0 1.5707963267948966" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_arm_urdf/meshes/kinect.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.07271 0.27794 0.073"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  <!-- The fixed joints & links below are usually published by static_transformers launched by the OpenNi launch 
		 files. However, for Gazebo simulation we need them, so we add them here.
		 (Hence, don't publish them additionally!) -->
  <joint name="camera_depth_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="camera_rgb_frame"/>
    <child link="camera_depth_frame"/>
  </joint>
  <link name="camera_depth_frame">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="camera_depth_optical_joint" type="fixed">
    <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
    <parent link="camera_depth_frame"/>
    <child link="camera_depth_optical_frame"/>
  </joint>
  <link name="camera_depth_optical_frame">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  <gazebo reference="camera_link">
    <material>Gazebo/Black</material>
  </gazebo>
</robot>
```

## 生成AR码Model

`git clone https://github.com/mikaelarguedas/gazebo_models.git`

参考其README

移动到`/ar_tags/scripts`

执行命令格式如下：

```shell
$ ./generate_markers_model.py -h
usage: generate gazebo models for AR tags

optional arguments:
  -h, --help            show this help message and exit
  -i IMAGES_DIR, --images-dir IMAGES_DIR
                        directory where the marker images are located
                        (default: $HOME/gazebo_models/ar_tags/images)
  -g GAZEBODIR, --gazebodir GAZEBODIR
                        Gazebo models directory (default:
                        $HOME/.gazebo/models)
  -s SIZE, --size SIZE  marker size in mm (default: 500)
  -v, --verbose         verbose mode (default: False)
  -w WHITE_CONTOUR_SIZE_MM, --white-contour-size-mm WHITE_CONTOUR_SIZE_MM
                        Add white contour around images, default to no contour
                        (default: 0)

./generate_markers_model.py -i IMAGE_DIRECTORY -g GAZEBO_MODELS_DIRECTORY -s SIZE_IN_MILLIMETER -w CONTOUR_SIZE_IN_MM
```

执行以下命令：

```shell
./generate_markers_model.py  -s 90
```

可获得90x90的Ar markers model，模型文件默认保存目录为`$HOME/.gazebo/models`

## 建立launch文件

`ar_track_param.launch`

```XML
    <launch>
        <arg name="marker_size" default="9" /> 
        <arg name="max_new_marker_error" default="0.08" />
        <arg name="max_track_error" default="0.2" />
        <arg name="cam_image_topic" default="/camera1/rgb/image_raw" />
        <arg name="cam_info_topic" default="/camera1/rgb/camera_info" />
        <arg name="output_frame" default="/base_link" />
        <node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen">
            <param name="marker_size"           type="double" value="$(arg marker_size)" />
            <param name="max_new_marker_error"  type="double" value="$(arg max_new_marker_error)" />
            <param name="max_track_error"       type="double" value="$(arg max_track_error)" />
            <param name="output_frame"          type="string" value="$(arg output_frame)" />
            <remap from="camera_image"  to="$(arg cam_image_topic)" />
            <remap from="camera_info"   to="$(arg cam_info_topic)" />
        </node>
    </launch>
```

arg参数可适当修改

## 建立py文件

`moveit_track_demo.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
import tf
import threading
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseStamped, Pose

x = 0
y = 0
z = 0
ox = 0
oy = 0
oz = 0
zw = 0  
# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)               
# 初始化需要使用move group控制的机械臂中的arm group
arm = moveit_commander.MoveGroupCommander('arm_group')
# 初始化需要使用move group控制的机械臂中的gripper group
gripper = moveit_commander.MoveGroupCommander('hand_group')       
# 获取终端link的名称
end_effector_link = arm.get_end_effector_link()                       
# 设置目标位置所使用的参考坐标系
reference_frame = 'base_link'
arm.set_pose_reference_frame(reference_frame)              
# 当运动规划失败后，允许重新规划
arm.allow_replanning(True)      
# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.05)
gripper.set_goal_joint_tolerance(0.001)        
# 控制机械臂先回到初始化位置
#arm.set_named_target('home')
#arm.go()
# 设置机器臂当前的状态作为运动初始状态
arm.set_start_state_to_current_state()
target_pose = PoseStamped()
a = 1 
def Listener():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/ar_pose_marker",AlvarMarkers,ar_pose, queue_size=1)
        rospy.spin()
def ar_pose(data):
        x = data.markers[0].pose.pose.position.x
        y = data.markers[0].pose.pose.position.y
        z = data.markers[0].pose.pose.position.z
        ox = data.markers[0].pose.pose.orientation.x
        oy = data.markers[0].pose.pose.orientation.y
        oz = data.markers[0].pose.pose.orientation.z
        ow = data.markers[0].pose.pose.orientation.w
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x =  x-0.08
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z+0.03
        target_pose.pose.orientation.x = 0.911822
        target_pose.pose.orientation.y = -0.0269758
        target_pose.pose.orientation.z = 0.285694
        target_pose.pose.orientation.w = -0.293653
        print(target_pose)
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        # rospy.sleep(1)
        global a
        a+=1
        print(" count ",a) 
        # 关闭并退出moveit
        #moveit_commander.roscpp_shutdown()
        #moveit_commander.os._exit(0)

        print("清除") 
if __name__ == "__main__":
    Listener()
```

## 执行

`roslaunch moveit_ros_robot_arm full_robot_arm_sim.launch `

`roslaunch moveit_progect ar_track_param.launch`

`rosrun moveit_progect moveit_track_demo.py `

演示视频: [https://www.bilibili.com/video/BV1k8411S7fo/?spm_id_from=333.999.0.0&vd_source=b57e293dfa3402722a1522f3d1c08c97](https://www.bilibili.com/video/BV1k8411S7fo/?spm_id_from=333.999.0.0&vd_source=b57e293dfa3402722a1522f3d1c08c97)

![image-20230722171206584](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/1aa6926dcf2c01da69e3fdd1d0ae663e.png)

参考文章:
[ROS机械臂控制之跟踪二维码](https://www.guyuehome.com/6873/feed)
