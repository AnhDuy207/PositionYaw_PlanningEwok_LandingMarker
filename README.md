# PositionYaw_PlanningEwok_LandingMarker
`*` Control drone by code Position + Yaw <br>
`*` Avoid obstacles by code Ewok <br>
`*` Landing by Marker <br>

## Installation
```
$ git@github.com:AnhDuy207/PositionYaw_PlanningEwok_LandingMarker.git
```

## Getting started simulation
1. Launch a quadrotor with px4 and mavros in gazebo 
```
$ roslaunch px4 mavros_posix_sitl.launch 
```
2. Run code Ewok create map
```
$ roslaunch ewok_optimization optimization_point.launch 
```
3. Run the offboard_position_yaw_control
```
$ roslaunch offboard plannerMarker.launch 
```
4. Run code detect Marker
```
$ rosrun offboard MarkerDetection.py
```

## Getting started practice
1. Launch a quadrotor with px4 and mavros in gazebo 
```
$ roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"
```
2. Run camera D435 & D455
```
$ roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=001622072448 depth_width:=640 depth_height:=480 depth_fps:=15
$ roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=146222251613 depth_width:=640 depth_height:=480 depth_fps:=15
```
3. Convert depth topic by python
```
$ cd duy_ws/src
$ python visualize_data.py
```
4. Run code Ewok create map
```
$ roslaunch ewok_optimization optimization_point.launch 
```
5. Run code detect Marker
```
$ rosrun offboard MarkerDetection.py
```
6. Run the offboard_position_yaw_control
```
$ roslaunch offboard plannerMarker.launch 
```
7. Record data
```
$ rosbag record -O subset_11012023 rostopic echo /cam_1/color/camera_info /cam_2/color/camera_info /cam_1/color/image_raw /cam_2/color/image_raw /cam_2/depth/image_rect_raw /depth_topic_2 /mavros/local_position/pose /mavros/imu/data /mavros/local_position/odom /ring_buffer/distance /ring_buffer/free /ring_buffer/free_array /ring_buffer/occupied /before_optimization /after_optimization /global_trajectory /tf /tf_static 
```
* Video demo: [Video](https://husteduvn-my.sharepoint.com/:v:/g/personal/quang_nguyenanh_hust_edu_vn/ET00gwt5OQ5Jm9ZKs9PFqUMBmEKejd-n5vpTubCI6PtrSg?e=TT036x)
