#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.height)

def img_callback(data):
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.height)

def cloud_callback(data):
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.height)

     
def listener():
  # In ROS, nodes are uniquely named. If two nodes with the same
  # name are launched, the previous one is kicked off. The
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.
  rospy.init_node('listener', anonymous=True)

  #rospy.Subscriber("/camera/points", PointCloud2, cloud_callback)
  rospy.Subscriber("/depth_registered/image_rect", Image, img_callback)
  rospy.Subscriber("/rgb/camera_info", CameraInfo, callback)
  #rospy.Subscriber("/depth/camera_info", CameraInfo, callback)


  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':
  listener()
