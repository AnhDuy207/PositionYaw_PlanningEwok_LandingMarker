# PositionYaw_PlanningEwok_LandingMarker
`*` Control drone by code Position + Yaw <br>
`*` Avoid obstacles by code Ewok <br>
`*` Landing by Marker <br>

## Installation
```
git@github.com:AnhDuy207/PositionYaw_PlanningEwok_LandingMarker.git
```

## Getting started
1. Launch a quadrotor with px4 and mavros in gazebo 
```
roslaunch px4 mavros_posix_sitl.launch 
```
2. Run code Ewok create map
```
roslaunch ewok_optimization optimization_point.launch 
```
3. Run the offboard_position_yaw_control
```
roslaunch offboard plannerMarker.launch 
```
4. Run code detect Marker
```
rosrun offboard MarkerDetection.py
```
