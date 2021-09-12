import rospy 
from math import pi
import tf
from moveit_planning.srv import Grasp, GraspRequest, MoveToPose, MoveToPoseRequest, MoveToJointTarget, MoveToJointTargetRequest
from std_srvs.srv import Empty

class Arm(object):

  # Initialize the service clients.
  def __init__(self):
    self.grasp_client = rospy.ServiceProxy("/grasp", Grasp)
    self.move_to_pose_client = rospy.ServiceProxy("/move_to_pose", MoveToPose)
    self.move_to_joint_target_client = rospy.ServiceProxy("/move_to_joint_target", MoveToJointTarget)
    self.close_fingers_client = rospy.ServiceProxy("/close_fingers", Empty)
    self.open_fingers_client = rospy.ServiceProxy("/open_fingers", Empty)


  # Call the service client for closing the fingers.
  def close_fingers(self):
    try:
      self.close_fingers_client()
      return True
    except:
      return False

  # Call the service client for opening the fingers, and removing attached collision objects.
  def open_fingers(self):
    try:
      self.open_fingers_client()
      return True
    except:
      return False

  # Call the service client to move the end effector to a given pose (x, y, z, roll pitch, yaw).
  def move_arm_to(self, x, y, z, roll, pitch, yaw):
    move_req = MoveToPoseRequest()
    move_req.pose.header.frame_id = "panda_link0"
    move_req.pose.header.stamp = rospy.Time.now()
    move_req.pose.pose.position.x = x
    move_req.pose.pose.position.y = y
    move_req.pose.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)   
    move_req.pose.pose.orientation.x = quaternion[0]
    move_req.pose.pose.orientation.y = quaternion[1]
    move_req.pose.pose.orientation.z = quaternion[2]
    move_req.pose.pose.orientation.w = quaternion[3]

    try:
      response = self.move_to_pose_client(move_req) 
    except:
      return False

    if response.success:
      return True 
    
    return False 

  # Call the service client to grasp the given bounding box. 
  def grasp(self, bounding_box):
    grasp_req = GraspRequest()
    grasp_req.bounding_box = bounding_box
    try:
      result = self.grasp_client(grasp_req)
      return True
    except:
      return False

  # A fixed position given joint positions, to make the end effector look left for objects
  def look_left(self):
    joint_req = MoveToJointTargetRequest() # Joint targets in radians
    joint_req.joint1 = 1.6019258968560814
    joint_req.joint2 = -1.5698311194643857
    joint_req.joint3 = -1.3050121486444723
    joint_req.joint4 = -2.21046597678035
    joint_req.joint5 = -0.9495575183376657 
    joint_req.joint6 = 1.7484533384368035
    joint_req.joint7 = -0.08913406365820009

    #try:
    response = self.move_to_joint_target_client(joint_req) 
    #except:
    #  return False

    if response.success:
      return True 
    
    return False

  # Move the End Effector above the blue dropbox, and open the fingers
  def drop_blue(self):
    if self.move_arm_to(-0.12, -0.45, 0.25, pi, -pi/2, -pi/2):
      return self.open_fingers()
    
    return False

  # Move the End Effecor above the red dropbox, and open the fingers
  def drop_red(self):
    if self.move_arm_to(0.28, -0.45, 0.25, pi, -pi/2, -pi/2):
      return self.open_fingers()

    return False

  # Move the End Effector above the green dropbox, and open the fingers
  def drop_green(self):
    if self.move_arm_to(0.08, -0.45, 0.25, pi, -pi/2, -pi/2):
      return self.open_fingers()
 
    return False
    
