#!/usr/bin/python3
import rospy
from bounding_box import BoundingBox
from object_spawner import ObjectSpawner
from object_recognition.msg import ObjectRecognitionAction, ObjectRecognitionGoal
from arm import Arm
from math import pi
import actionlib

rospy.init_node("testing_grasping_node")
bounding_box = BoundingBox() # For receiving the bounding boxes
object_spawner = ObjectSpawner() # For (de)spawning the objects 
object_recognition_client = actionlib.SimpleActionClient("/object_recognition", ObjectRecognitionAction) 
print("Waiting for object_recognition server")
object_recognition_client.wait_for_server()
print("Connected")

arm = Arm() # For giving commands to the MoveIt node

while True: # Just endlessly keep going until no objects are found
  rospy.sleep(1)
  object_spawner.spawn_random_object() # Remove and spawn a new random object (either blue, red or green box)
  rospy.sleep(1)

  while not arm.look_left(): # Try to keep moving to the 'Look left' position. Most likely fails if octomap got update during planning
    rospy.sleep(1)
    print("Move failed, try again")

  bounding_boxes = bounding_box.get_bounding_box() # Get the bounding boxes (should only be 1)

  if len(bounding_boxes) == 0:
    print("No bounding boxes left")
    break # Done
  
  goal_msg = ObjectRecognitionGoal() # Empty goal message for starting object recognition
  object_recognition_client.send_goal_and_wait(goal_msg)
  state = object_recognition_client.get_state()

  if state == actionlib.GoalStatus.ABORTED:
    print("No objects found")
    break # Done
  
  result = object_recognition_client.get_result()
  object_name = result.objects[0].label # Should only be 1 object, use the label to know where to drop it.

  # Grasp the object (given the bounding box data)
  if arm.grasp(bounding_boxes[0]) == True:
    print("Grasp successfull")
  else:
    print("Grasp failed, still going to drop zone") # Sometimes weird things with Gazebo happen, so still attempt a drop
    #failed = True
    #continue

  # Based on object recognition, move to the correct color dropbox
  if object_name == "redbox":
    while not arm.drop_red():
      rospy.sleep(1)
      print("Moving to red dropbox failed")
  elif object_name == "bluebox":
    while not arm.drop_blue():
      rospy.sleep(1)
      print("Moving to blue dropbox failed")
  elif object_name == "greenbox":
    while not arm.drop_green():
      rospy.sleep(1)
      print("Moving to green dropbox failed")

  rospy.sleep(1) 
  arm.close_fingers() # Close the fingers after dropping object
  
