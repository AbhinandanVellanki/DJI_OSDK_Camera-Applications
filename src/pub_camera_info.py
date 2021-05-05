#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header


def pub_info(main_info, depth_info):
	main_info_pub = rospy.Publisher('/rgb/camera_info', CameraInfo, queue_size=10)
	depth_info_pub = rospy.Publisher('/depth/camera_info', CameraInfo, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		main_info_pub.publish(main_info)
		depth_info_pub.publish(depth_info)
		rate.sleep()

def create_mesg(frame_id, height, width, distortion_model, D, K, R, P):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = frame_id

	cam_msg = CameraInfo()
	cam_msg.header = header
	cam_msg.height = height
	cam_msg.width = width
	cam_msg.distortion_model = distortion_model
	cam_msg.D = D
	cam_msg.K = K
	cam_msg.R = R
	cam_msg.P = P

	return cam_msg

def main(stereo_h, stereo_w, stereo_D, stereo_K, stereo_R, stereo_P, main_h, main_w, main_D, main_K, main_R, main_P):
	rospy.init_node('pub_info', anonymous=True)
	main_info = create_mesg("/rgb_optical_frame", stereo_h, stereo_w, "plumb_bob", stereo_D, stereo_K, stereo_R, stereo_P)
	stereo_info = create_mesg("/depth_optical_frame", main_h, main_w, "plumb_bob", main_D, main_K, main_R, main_P)
	pub_info(main_info, stereo_info)

if __name__ == '__main__':
	#Set stereo camera parameters
	stereo_h = 480
	stereo_w = 640
	stereo_D = [0.031148, -0.037259, -0.002853, 0.000446, 0.000000]
	stereo_K = [462.887168, 0.000000, 313.915571, 0.000000, 463.694275, 233.443807, 0.000000, 0.000000, 1.000000]
	stereo_R = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
	stereo_P = [466.333618, 0.000000, 314.173672, 0.000000, 0.000000, 466.858032, 232.156909, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]

	#Set main camera parameters
	main_h = 720
	main_w = 1280
	main_D = [-0.108081, 0.080274, -0.002777, 0.001018, 0.000000]
	main_K = [1046.865149, 0.000000, 651.99872, 0.000000, 1051.659853, 331.693277, 0.000000, 0.000000, 1.000000]
	main_R = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
	main_P = [1015.268921, 0.000000, 653.766007, 0.000000, 0.000000, 1038.475708, 330.004477, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]

	try:
		main(stereo_h, stereo_w, stereo_D, stereo_K, stereo_R, stereo_P, main_h, main_w, main_D, main_K, main_R, main_P)
	except rospy.ROSInterruptException:
		pass
		
