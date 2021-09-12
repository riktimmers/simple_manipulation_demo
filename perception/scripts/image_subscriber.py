#!/usr/bin/python3
import rospy 
from sensor_msgs.msg import Image

def image_callback(imsg_msg):
  pass

rospy.init_node("image_subscriber")
sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.spin()
