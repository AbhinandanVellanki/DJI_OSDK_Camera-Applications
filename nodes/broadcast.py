#!/usr/bin/env python
import roslib
import tf
import rospy
#from geometry_msgs.msg import Quaternion
from pyquaternion import Quaternion
import math

# def get_Transform():
# 	rot_mat = tf.linalg.Matrix3x3(-0.9473512, 0.17987305, 0.26489883, -0.15806205, -0.98218273, 0.10165367, 0.27846382, 0.05443128, 0.95890309)
# 	tran_vec = tf.Vector3(-2.36747364, 9.89318386, -7.28534878)
# 	transform = tf.Transform(rot_mat, tran_vec)

# 	return transform

def bcast(tran_vec, rot_quaternion, parent_frame, child_frame):
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10)
	br.sendTransform((tran_vec[0], tran_vec[1], tran_vec[2]), rot_quaternion, rospy.Time.now(), child_frame, parent_frame)

def rot_mat_to_quaternion(m00, m01, m02, m10, m11, m12, m20, m21, m22):
	# if m22 < 0:
	#     if m00 >m11:
	#         t = 1 + m00 -m11 -m22
	#         q = Quaternion(t, m01+m10, m20+m02, m12-m21)
	#     else:
	#         t = 1 -m00 + m11 -m22
	#         q = Quaternion(m01+m10, t, m12+m21, m20-m02)
	# else:
	#     if m00 < -m11: 
	#         t = 1 -m00 -m11 + m22
	#         q = Quaternion(m20+m02, m12+m21, t, m01-m10)
	#     else:
	#         t = 1 + m00 + m11 + m22
	#         q = Quaternion(m12-m21, m20-m02, m01-m10, t)
	# q *= 0.5 / math.sqrt(t)
	trace = m00 + m11 + m22

	if trace > 0.0:
		k = 0.5/math.sqrt(1.0+trace)
		q = Quaternion(k*(m12-m21), k*(m20-m02), k*(m01-m10), 0.25/k)
	elif m00 > m11 and m00 > m22:
		k = 0.5/math.sqrt(1.0+m00-m11-m22)
		q = Quaternion(0.25/k, k*(m10+m01), k*(m20+m02), k*(m12-m21))
	elif m11 > m22:
		k = 0.5/math.sqrt(1.0+m11-m00-m22)
		q = Quaternion(k*(m10+m01), 0.25/k, k*(m21+m12), k*(m20-m02))
	else:
		k = 0.5/math.sqrt(1.0+m22-m00-m11)
		q = Quaternion(k*(m20+m02), k*(m21+m12), 0.25/k, k*(m01-m10))
	return q

if __name__ == '__main__':

	#define transformation from world to main camera
	tran_vec_w2m = [-4.62572233, -3.43853233, 33.43460906] #translation vector from depth frame to rgb frame
	rot_vec_w2m = [-0.11909564, 0.43927222, 1.47500789] #rotation vector from depth frame to rgb frame
	rot_quaternion_w2m = rot_mat_to_quaternion(0.03295435, -0.97655179, 0.21274541, 0.93383356, 0.10594441, 0.34165871, -0.35618661, 0.18740966, 0.91542816)

	#define transformation from world to depth camera
	tran_vec_w2d = [-6.99319597, 6.45465153, 26.14926028]
	rot_vec_w2d = [0.14586201, 0.29524993, -1.50140133]
	rot_quaternion_w2d = rot_mat_to_quaternion(0.04239873, 0.99383865, 0.10240643, -0.9586117, 0.06934989, -0.27614162, -0.2815421, -0.08645994, 0.95564571)

	# #define transformation from depth camera to main camera
	# tran_vec_d2m = [0.68003865, -14.18108947, 6.2781987]
	# rot_vec_d2m = [-0.92316412, 0.16224135, 2.81822511]
	# rot_quaternion_d2m = rot_mat_to_quaternion(-0.79350503, -0.19572782, -0.57622946, 0.12830843, -0.97939111, 0.15598074, -0.59488378, 0.0498364, 0.8022653)
	tran_vec_d2m = [0, 0, 0]
	rot_quaternion_d2m = rot_mat_to_quaternion(1, 1, 1, 1, 1, 1, 1, 1, 1)

	try:
		rospy.init_node('stereo_z30')
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			#bcast(tran_vec_w2m, rot_quaternion_w2m, "world", "/rgb_optical_frame")  #broadcast transform from "world" to "/rgb_optical_frame"
			#bcast(tran_vec_w2d, rot_quaternion_w2d, "world", "/depth_optical_frame")  #broadcast transform from "world" to "/depth_optical_frame"
			bcast(tran_vec_d2m, rot_quaternion_d2m, "/depth_optical_frame", "/rgb_optical_frame") #broadcast from "/depth_optical_frame" to "/rgb_optical_frame"
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
