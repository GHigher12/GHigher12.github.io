---
title: ROS-Moveit和Gazebo联合仿真（二）
typora-root-url: ..\imgs
date: 2023-07-13 16:58:13
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

## URDF功能包配置

### config

首先在SW2URDF生成的功能包下Config目录下新建文件`joint_trajectory_controller.yaml`

```yaml
robot_arm_controller:
  type: "position_controllers/JointTrajectoryController"
  joints: [joint1, joint2, joint3, joint4, joint5, joint6]

hand_ee_controller:
  type: "position_controllers/JointTrajectoryController"
  joints: [joint_hand1, joint_hand2]

joint_state_controller:
  type: "joint_state_controller/JointStateController"
  publish_rate: 50 
```

### launch

在launch文件目录下新建`arm_urdf.launch`文件

```xml
<launch>
	<arg name="arg_x" default="0.00" />
	<arg name="arg_y" default="0.00" />
	<arg name="arg_z" default="0.00" />
	<arg name="arg_R" default="0.00" />
	<arg name="arg_P" default="0.00" />
	<arg name="arg_Y" default="0.00" />

	<!--Urdf file path-->
	<param name="robot_description" textfile="$(find ros_robot_arm)/urdf/ros_robot_arm.urdf"/>

	<!--spawn a empty gazebo world-->
	<include file="$(find gazebo_ros)/launch/empty_world.launch" />
	<node name="tf_footprint_base" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0 base_link base_footprint 40" />

	<!--spawn model-->
	<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-x $(arg arg_x) -y $(arg arg_y) -z $(arg arg_z) -Y $(arg arg_Y) -param robot_description -urdf -model ros_robot_arm -J joint1 0.0 -J joint2 0.0 -J joint3 0.0 -J joint4 0.0 -J joint5 0.0 -J joint6 0.0 -J joint_hand1 0.0 -J joint_hand2 0.0" />

	<!--Load and launch the joint trajectory controller-->
	<rosparam file ="$(find ros_robot_arm)/config/joint_trajectory_controller.yaml" command="load"/>
	<node name= "controller_spawner" pkg= "controller_manager" type="spawner" respawn="false" output="screen" args="joint_state_controller robot_arm_controller hand_ee_controller"/>
    
	<!-- Robot State Publisher for TF of each joint: publishes all the current states of the joint, then RViz can visualize -->
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>
</launch>
```

### CMakeLists.txt

添加如下信息

```cmake
find_package(catkin REQUIRED
		message_generation
		roscpp
		rospy
		std_msgs
		geometry_msgs
		urdf
		xacro
		message_generation
)

catkin_package(CATKIN_DEPENDS
		geometry_msgs
		roscpp
		rospy
		std_msgs
)
```

### package.xml

```xml
<package format="2">
  <name>ros_robot_arm</name>
  <version>1.0.0</version>
  <description>
    <p>URDF Description package for ros_robot_arm</p>
    <p>This package contains configuration data, 3D models and launch files
for ros_robot_arm robot</p>
  </description>
  <author>TODO</author>
  <maintainer email="TODO@email.com" />
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
   
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>urdf</build_depend>
  <build_depend>xacro</build_depend>
  <build_depend>message_generation</build_depend>

  <depend>roslaunch</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>gazebo</depend>
  <depend>moveit_simple_controller_manager</depend>


  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>urdf</build_export_depend>
  <build_export_depend>xacro</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>message_runtime</exec_depend>

  <export>
    <architecture_independent />
  </export>
</package>
```



### urdf文件

为机器人的各个joint添加控制器

```xml
<transmission name="link1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link1_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="link2_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link2_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="link3_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint3">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link3_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="link4_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint4">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link4_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="link5_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint5">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link5_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="link6_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint6">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link6_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="link_hand1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_hand1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link_hand1_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="link_hand2_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_hand2">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link_hand2_motor">
      <mechanicalReduction>50</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <gazebo>
    <plugin name="control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>

<gazebo reference="base_link">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="Link1">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="Link2">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Turquoise</material>
  </gazebo>

  <gazebo reference="Link3">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Yellow</material>
  </gazebo>

  <gazebo reference="Link4">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="Link5">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="Link6">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="Hand1">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="Hand2">
    <selfCollide>true</selfCollide>
    <material>Gazebo/Grey</material>
  </gazebo>
```



## Moveit功能包配置

### config

修改`ros_controllers.yaml`

```yaml
moveit_sim_hw_interface:
  joint_model_group: hand_group
  joint_model_group: hand_close

generic_hw_control_loop:
  loop_hz: 300
  cycle_time_error_threshold: 0.01

hardware_interface:
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
    - joint_hand1
    - joint_hand2
  sim_control_mode: 1 # 0: position, 1: velocity

joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50
controller_list:
  []
```

在config文件目录下新建`new_ros_controllers.yaml`

```yaml
# JointTrajectoryController
controller_list:
  - name: robot_arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
    - joint1
    - joint2
    - joint3
    - joint4 
    - joint5
    - joint6
  - name: hand_ee_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
    - joint_hand1
    - joint_hand2
```

### launch

修改launch文件目录下`simple_moveit_controller_manager.launch.xml`

将`ros_controllers.yaml`改为`new_ros_controllers.yaml`

```xml
<launch>
  <!-- Define the MoveIt controller manager plugin to use for trajectory execution -->
  <param name="moveit_controller_manager" value="moveit_simple_controller_manager/MoveItSimpleControllerManager" />

  <!-- Load controller list to the parameter server -->
  <rosparam file="$(find moveit_ros_robot_arm)/config/simple_moveit_controllers.yaml" />
  <rosparam file="$(find moveit_ros_robot_arm)/config/new_ros_controllers.yaml" />
</launch>
```

在launch文件目录下新建`full_robot_arm_sim.launch`

```xml
<launch>
	<!-- Launch Your robot arms launch file which loads the robot in Gazebo and spawns the controllers -->
	<include file = "$(find ros_robot_arm)/launch/arm_urdf.launch" />

	<!-- Launch Moveit Move Group Node -->
	<include file = "$(find moveit_ros_robot_arm)/launch/move_group.launch" />

	<!-- Run Rviz and load the default configuration to see the state of the move_group node -->
	<arg name="use_rviz" default="true" />
	<include file="$(find moveit_ros_robot_arm)/launch/moveit_rviz.launch" if="$(arg use_rviz)">
		<arg name="rviz_config" value="$(find moveit_ros_robot_arm)/launch/moveit.rviz"/>
	</include>

</launch>
```

## 运行

```sh
source ./devel/setup.bash
roslaunch moveit_ros_robot_arm full_robot_arm_sim.launch
```

首先确定最终姿态，然后`Plan`规划，再`Execute`执行，可看到`Gazebo`中机械臂开始运动

![image-20230707181422728](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/240066b63de04be15ea5973da8b3c622.png)
