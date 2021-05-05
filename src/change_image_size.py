#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2

s_im_data = Image()
m_im_data = Image()

def process_stereo(data):
	global s_im_data
	# s_im_data.header = data.header
	# s_im_data.encoding = data.encoding
	# s_im_data.is_bigendian = data.is_bigendian
	# s_im_data.step = data.step
	# s_im_data.height = data.height
	# s_im_data.width = data.width
	# s_im_data.data = data.data
	bridge = CvBridge()
	cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	cv_img = cv_img[0:415, 0:640]
	#print(cv_img.shape)
	s_im_data = bridge.cv2_to_imgmsg(cv_img, encoding="mono8")
	

def process_main(data):
	global m_im_data
	# m_im_data.header = data.header
	# m_im_data.encoding = data.encoding
	# m_im_data.is_bigendian = data.is_bigendian
	# m_im_data.step = data.step
	# m_im_data.height = data.height
	# m_im_data.width = data.width
	# m_im_data.data = data.data
	bridge = CvBridge()
	cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	cv_img = cv_img[0:415, 0:640]
	#print(cv_img.shape)
	m_im_data = bridge.cv2_to_imgmsg(cv_img, encoding='rgb8')

def changer():
	global s_im_data
	global m_im_data
	rospy.init_node('changer')
	rospy.Subscriber('/dji_osdk_ros/stereo_vga_front_left_images', Image, process_stereo)
	rospy.Subscriber('/dji_osdk_ros/main_camera_images', Image, process_main)
	pub_s = rospy.Publisher('/mycamera/stereo', Image, queue_size = 1000)
	pub_m = rospy.Publisher('/mycamera/main', Image, queue_size = 1000)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub_m.publish(m_im_data)
		#print(s_im_data)
		pub_s.publish(s_im_data)
		#rospy.spin()
		rate.sleep()

if __name__ == '__main__':
	try:
		changer()
	except rospy.ROSInterruptException:
		pass

