# jemaro_days_2025

The rosbag LiDAR dataset and the lane paths are available here: https://uncloud.univ-nantes.fr/index.php/s/tkEwQcM7qGMj9wp 

# Gazebo
The Gazebo prius environment is based on https://github.com/mattborghi/osrf_car_demo

Launch the Gazebo simulator with the straight road world and the prius:
```
ros2 launch car_demo straight_road.launch.py
```

Convert twist messages to the prius control messages:
```
ros2 launch twist_to_prius_cmd.launch.py
```

Launch a basic pure pursuit path following algorithm:
```
ros2 launch car_control car_control.launch.py
```

Publish the path of the center of the lane and of the right and left sides of the road (only one time, use -l for multiple messages):
```
ros2 bag play path_prius
```
```
ros2 bag play path_right_prius --remap /path:=path_right
```
```
ros2 bag play path_left_prius --remap /path:=path_left
```

# LiDAR dataset

Launch rviz interface:
```
rviz2 -d jemaro.rviz
```

Publish the path of the center of the lane and of the right and left sides of the road (only one time, use -l for multiple messages):
```
ros2 bag play path_zoe --remap /ZOE3/path_follower/setPath:=path
```
```
ros2 bag play path_left_zoe --remap /ZOE3/path_follower/setPath:=path_left
```
```
ros2 bag play path_right_zoe --remap /ZOE3/path_follower/setPath:=path_right
```

Read the LiDAR dataset:
```
ros2 bag play rosbag2_2025_06_26-10_27_18
```

Launch the pointcloud downsampling node (based on https://github.com/LihanChen2004/pointcloud_downsampling):
```
ros2 launch pointcloud_downsampling pointcloud_downsampling.launch.py
```

rosbag lanes
rosbag pointcloud

rl lane_switch
rr object_detection
rm clean_point_cloud


