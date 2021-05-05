#!bin/bash

params_file_path=${1}

roslaunch dji_osdk_ros dji_vehicle_node.launch  #run OSDK-ROS
rosrun stereo_vision_depth_perception_node $params_file_path #run stereo depth perception application