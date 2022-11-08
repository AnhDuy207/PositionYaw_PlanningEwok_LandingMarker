# PositionYaw_PlanningEwok_LandingMarker
`*` Control drone by code Position + Yaw in real environment<br>

## Create ws
```
$ mkdir -p duy_ws/src
$ cd duy_ws/src
$ catkin_init_workspace
```

## Installation
```
$ cd duy_ws/src
$ git clone git@github.com:AnhDuy207/planning_Gazebo_ws.git
$ cd to the folder containing the git folder
$ git checkout positionYaw
$ git pull
```

## Getting started
1. Launch a quadrotor with px4 and mavros 
```
$ roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"
```
2. Run the offboard_position_yaw_control
```
$ roslaunch offboard plannerMarker.launch
```
3. Recoding data
```
$ rosbag record -O name_bag /camera/color/image_raw /camera/depth/image_rect_raw /depth_topic_2 /mavros/local_position/pose /mavros/imu/data /mavros/local_position/odom
```
* Video demo: [Video](https://husteduvn-my.sharepoint.com/:v:/g/personal/quang_nguyenanh_hust_edu_vn/ESR3Qivc1TxBvH5o7vi7_8MBmjZ3Hbdy8ZbDBY1d8joO9A?e=lsLx1c)
