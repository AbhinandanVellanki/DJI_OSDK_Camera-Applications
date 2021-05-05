# DJI_OSDK_Camera-Applications

## For running applications on DJI OSDK-ROS using Manifold-2C Onbaord computer on M210 RTk v2 Drone

### DJI Matrice M210 RTK v2 : https://www.dji.com/hk-en/matrice-200-series-v2/info#specs

### DJI Manifold-2C: https://www.dji.com/hk-en/manifold-2/specs

### Clone and build DJI OSDK-ROS : https://github.com/dji-sdk/Onboard-SDK-ROS 

## Applications for:

1. Calibrating Stereo Cameras 
2. Stereo Depth Perception
3. Calibrating Stereo Camera with Payload Camera
4. Publishing Camera Frame Transformation on ROS
5. Publishing Camera Parameters (Extrinsic and Intrinsic) and information on ROS
6. Projecting 3D Pointcloud to Payload Camera Frame


## Running Instructions:

### Calibrating Stereo Cameras:

1. Run the built OSDK-ROS:

`$roslaunch dji_osdk_ros dji_vehicle_node.launch`

2. Print a checkerboard pattern for calinbration. For this project, an A4 size 9x7 checkerboard with 25mm squares was used.

3. Run ROS camera calibration node:

`$rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.025 right:=dji_osdk_ros/stereo_vga_front_right_images left:=dji_osdk_ros/stereo_vga_front_left_images`

4. Save the calibration to file (see scripts/m210_stereo_param.yaml for reference)

### Stereo Depth Perception:

1. navigate to scripts/

2. run `$./depth_perception.sh <calibration_params_relative_filepath>`

3. Point cloud will be published as sensor_msgs/PointCloud2 messages on ROS Topic: /stereo_depth_perception/unprojected_pt_cloud

### Calibrating stereo camera with payload cameras:

1. Collect images of a checkerboard taken simultaneously from both cameras. The ROS camera calibration package can be used to collect images, since it ensures all major angles and positions are captured.

2. Save the images in separate folders and edit lines 213-217 of src/calib_extrinsic.py with the camera and image folder names

3. Run `$rosrun dji_osdk_apps calib_extrinsic.py` to save the calibration parameters to text files

### Publish Camera Frame Transformation:

1. Edit transformation matrices in nodes/broadcast.py lines 56-63 with the values obtained after calibration

2. Run `$catkin_make` 

3. Run `$roslaunch dji_osdk_apps broadcast_transform.launch` 

4. Verify by running `$rqt_graph` or `$rosrun tf view_frames` and viewing saved pdf

### Publishing Camera Parameters (Extrinsic and Intrinsic) and information on ROS:

1. Edit camera information and parameters in line 43-56 of src/pub_camera_info.py and desired ROS Topic names for publisher in lines 10-11

2. Run `$catkin_make`

3. Run `$roslaunch dji_osdk_apps publish_cam_info.launch`

4. The camera information will be published as sensor_msgs/CameraInfo messages on defined ROS topics

### Projecting 3D Pointcloud to Payload Camera Frame:

1. Edit Relative Extrinsic Matrix in lines 47-58 and desried ROS topic name for publisher in line 61 of file reproject.cpp 

2. Run `$rosrun dji_osdk_apps reproject`

3. Projected Point Cloud will be published as sensor_msgs/PointCloud2 messages on the defined ROS topic

