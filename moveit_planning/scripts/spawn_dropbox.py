#!/usr/bin/env python3
import rospy 
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import tf
import rospkg
from math import pi
from random import shuffle, uniform

# Script for spawning the 3 color dropboxes at a fixed location, should be called only once at Gazebo startup

# Spawn the object at the given location
def spawn_object(object_name, x, y, z, rotation):
    path = rospack.get_path("perception")
    with open(path +  "/sdf/" + str(object_name) + ".sdf") as f:
        box_xml = f.read()
        
    orientation = Quaternion()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, rotation)
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]
    
    pose = Pose(Point(x, y, z), orientation)
    
    spawn_model(object_name, box_xml, "", pose, "world")            

# Spawn the dropboxes at a given fixed location        
if __name__ == "__main__":
    rospy.init_node("set_objects")
    rospack = rospkg.RosPack()
    print('Waiting for service...')
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print('Connected to service')
    
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    

    delete_model("red_dropbox")
    delete_model("green_dropbox")
    delete_model("blue_dropbox")

    
    spawn_object("red_dropbox", 0.2, -0.5, 0.07, 0.0)
    spawn_object("green_dropbox", 0.0, -0.5, 0.07, 0.0)
    spawn_object("blue_dropbox", -0.2, -0.5, 0.07, 0.0)
    
    print('Done')
