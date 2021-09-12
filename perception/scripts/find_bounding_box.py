import rospy
from perception.srv import FindBoundingBox, FindBoundingBoxRequest
from moveit_planning.srv import AddBox, AddBoxRequest

rospy.init_node("find_bounding_box_node")

get_bounding_box_client = rospy.ServiceProxy("/get_bounding_box", FindBoundingBox)
add_box_client = rospy.ServiceProxy("/add_box", AddBox)

find_bounding_box_msg = FindBoundingBoxRequest()
find_bounding_box_msg.transform_to = "arm_link0"
find_bounding_box_msg.area.min_x = -1
find_bounding_box_msg.area.max_x = 1
find_bounding_box_msg.area.min_y = -1
find_bounding_box_msg.area.max_y = 1
find_bounding_box_msg.area.min_z = -1
find_bounding_box_msg.area.max_z = 1
find_bounding_box_msg.filter_option.cluster_threshold = 0.05
find_bounding_box_msg.filter_option.leaf_size = 0.005
find_bounding_box_msg.filter_option.plane_threshold = 0.01

try:
  result = get_bounding_box_client(find_bounding_box_msg)
except rospy.ServiceException as e:
  print("Service call failed:", e)
  exit()

for index, bounding_box in enumerate(result.bounding_boxes):
  add_box_msg = AddBoxRequest()
  add_box_msg.frame_id = "arm_link0"
  add_box_msg.name = "box" + str(index)
  add_box_msg.width = bounding_box.width
  add_box_msg.length = bounding_box.length
  add_box_msg.height = bounding_box.height
  add_box_msg.x = bounding_box.x
  add_box_msg.y = bounding_box.y
  add_box_msg.z = bounding_box.z
  add_box_msg.yaw = bounding_box.yaw
  add_box_msg.pitch = bounding_box.pitch
  add_box_msg.roll = bounding_box.roll

  print(bounding_box.z, bounding_box.height)

  response = add_box_client(add_box_msg)