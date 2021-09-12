import rospy
from moveit_planning.srv import MoveToPose, MoveToPoseRequest, MoveToPoseResponse
from geometry_msgs.msg import PoseStamped
import tf
import math
from math import pi

## Simple script for testing the Move To Pose service
rospy.init_node("move_to_pose")
client = rospy.ServiceProxy("move_to_pose", MoveToPose)

print("Waiting for move_to_pose service")
client.wait_for_service()
print("Connected")

pose = PoseStamped()
pose.header.frame_id = "panda_link0"
pose.header.stamp = rospy.Time.now()

pose.pose.position.x = 0.35 
pose.pose.position.y = 0.0
pose.pose.position.z = 0.27
quaternion = tf.transformations.quaternion_from_euler(pi, -pi/6, pi/2) # roll, pitch, yaw
pose.pose.orientation.x = quaternion[0]
pose.pose.orientation.y = quaternion[1]
pose.pose.orientation.z = quaternion[2]
pose.pose.orientation.w = quaternion[3]

move_to_pose_msg = MoveToPoseRequest()
move_to_pose_msg.pose = pose

response = client(move_to_pose_msg)

if response.success: 
  print "succes"
else:
  print "failed"


