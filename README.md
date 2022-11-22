# PositionYaw_PlanningEwok_LandingMarker
`*` Control drone by code Position + Yaw in real environment<br>
`*` Planning by code Ewok in real environment<br>

## Build on Jeson nano
1. Install some necessary packages
```
$ sudo apt-get install ros-melodic-octomap
$ sudo apt-get install ros-melodic-octomap-*
$ sudo apt-get install ros-melodic-vision-opencv
$ sudo apt-get install ros-melodic-sophus
$ sudo apt-get install libsuitesparse-dev
$ sudo apt-get install libnlopt-dev
$ sudo apt-get install libnlopt-cxx-dev
```
2. Modify file /home/duynguyen/planning_Gazebo_ws/src/ewok_optimization/CMakeLists.txt
```
include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

catkin_simple()
```
to
```
include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})
set(CHOLMOD_LIBRARY "/usr/lib/aarch64-linux-gnu/libcholmod.so")
catkin_simple()
```
3. Modify file /home/duynguyen/planning_Gazebo_ws/src/ewok_ring_buffer/CMakeLists.txt
```
find_package(tf_conversions REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR} ${OCTOMAP_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(ring_buffer_example src/ring_buffer_example.cpp)
cs_add_executable(tum_rgbd_ring_buffer_example src/tum_rgbd_ring_buffer_example.cpp)
target_link_libraries(tum_rgbd_ring_buffer_example ${OCTOMAP_LIBRARIES})

catkin_add_gtest(test_ring_buffer_base test/ring-buffer-base-test.cpp)
```
to
```
find_package(tf_conversions REQUIRED)
find_package(OpenCV REQUIRED)

#set(OPENCV_INCLUDE_DIRS "/usr/include/opencv4")

include_directories(${EIGEN3_INCLUDE_DIR} ${OCTOMAP_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
#target_include_directories(tum_rgbd_ring_buffer_example ${OPENCV_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(ring_buffer_example src/ring_buffer_example.cpp)
cs_add_executable(tum_rgbd_ring_buffer_example src/tum_rgbd_ring_buffer_example.cpp)
target_link_libraries(tum_rgbd_ring_buffer_example ${OCTOMAP_LIBRARIES})
target_link_libraries(tum_rgbd_ring_buffer_example ${OpenCV_LIBS})


catkin_add_gtest(test_ring_buffer_base test/ring-buffer-base-test.cpp)
```

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
$ git checkout simulation_D435
$ git pull
```

## Getting started
1. Launch a quadrotor with px4 and mavros 
```
$ roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"
```
2. Run camera D435
```
$ roslaunch realsense2_camera rs_camera.launch depth_width:=640 depth_height:=480 depth_fps:=15
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
5. Run the offboard_position_yaw_control
```
$ roslaunch offboard plannerMarker.launch
```
6. Recoding data
```
$ rosbag record -O name_bag /camera/color/image_raw /camera/depth/image_rect_raw /depth_topic_2 /mavros/local_position/pose /mavros/imu/data /mavros/local_position/odom /ring_buffer/distance /ring_buffer/free /ring_buffer/free_array /ring_buffer/occupied /before_optimization /after_optimization /global_trajectory
```
* Video demo: [Video](https://husteduvn-my.sharepoint.com/:v:/g/personal/quang_nguyenanh_hust_edu_vn/EeRGW7vAgqpLhzBoNuNxrcABJUeIffA_Xx9K0w52rI71Bw?e=q3N6Qc)
