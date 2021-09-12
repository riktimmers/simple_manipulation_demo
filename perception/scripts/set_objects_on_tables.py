#!/usr/bin/python3 
import rospy 
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import tf
import rospkg
from math import pi
from random import shuffle, uniform


def SpawnObject(object_name, x, y, z, rotation):
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
        
if __name__ == "__main__":
    rospy.init_node("set_objects")
    rospack = rospkg.RosPack()
    print('Waiting for service...')
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print('Connected to service')
    
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    
    pos1 = [0.4, 0.4]
    objects = ["box0"]
    
    for object in objects:
        delete_model(object)
    
    
    
    SpawnObject(objects[0], pos1[0], pos1[1], 0.77, uniform(0, pi))
    
    print('Done')