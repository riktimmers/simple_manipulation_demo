import rospy 
from perception.srv import FindBoundingBox, FindBoundingBoxRequest

class BoundingBox(object):

  # Initialize the bounding box service client
  def __init__(self):
    self.get_bounding_box_client = rospy.ServiceProxy("/get_bounding_box", FindBoundingBox)

  # Call the service client to get the bounding boxes
  def get_bounding_box(self):
    # All values are in meters
    bounding_box_msg = FindBoundingBoxRequest()
    bounding_box_msg.area.min_x = 0.1
    bounding_box_msg.area.max_x = 1.0
    bounding_box_msg.area.min_y = 0.1
    bounding_box_msg.area.max_y = 1.0
    bounding_box_msg.area.min_z = -0.1 # Do not remove the floor with the cropping filter
    bounding_box_msg.area.max_z = 0.4
    bounding_box_msg.filter_option.cluster_threshold = 0.05
    bounding_box_msg.filter_option.leaf_size = 0.005
    bounding_box_msg.filter_option.plane_threshold = 0.01 
    bounding_box_msg.transform_to = "panda_link0"

    try:
      result = self.get_bounding_box_client(bounding_box_msg)
    except:
      return []

    return result.bounding_boxes