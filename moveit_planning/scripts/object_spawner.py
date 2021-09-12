from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
import tf 
import rospkg 
from math import pi 
from random import shuffle, uniform, randint
import rospy 

class ObjectSpawner(object):

  # Initialize the Gazebo service clients
  def __init__(self):
    rospack = rospkg.RosPack()
    self.path = rospack.get_path("perception") # Get the path of the package
  
    print("Waiting for gazebo service")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Connected to gazebo service")

    self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    # The names of the objects
    self.objects = ["greenbox", "bluebox", "redbox"]

    # Delete all objects (in case there is still 1 spawned)
    for obj in self.objects:
      self.delete_model(obj)

  # Spawn a random object around (0.45, 0.3) with a random orientation 
  def spawn_random_object(self):
    for obj in self.objects: # Delete all objects
      self.delete_model(obj)

    shuffle(self.objects) # Shuffle the names of objects
    object_name = self.objects[0] # Pick the first object (which are shuffled)
    x = 0.45 + uniform(-0.1, 0.1) # x location +/- 0.1m
    y = 0.3 + uniform(-0.05, 0.05) # y location +/- 0.05m
    z = 0.1 # Spawn a bit in the air so it will drop on the floor
    yaw = uniform(0, pi) # Random rotation between 0 and 180 degrees 

    # Open the xml sdf data file 
    with open(self.path + "/sdf/" + str(object_name) + ".sdf") as f:
      box_xml = f.read()

    orientation = Quaternion() # Convert yaw angle to Quaternion
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]

    pose = Pose(Point(x, y, z), orientation)
    self.spawn_model(object_name, box_xml, "", pose, "world") # Spawn the object
