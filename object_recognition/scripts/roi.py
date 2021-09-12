import rospy 
from perception.srv import FindROIs, FindROIsRequest

class ROI(object):

  # Initialize the roi service client
  def __init__(self):
    self.find_roi_client = rospy.ServiceProxy("/get_rois", FindROIs)
  
  def get_rois(self):
    request = FindROIsRequest()
    request.transform_to = "panda_link0"
    # all values in meters
    request.filter_option.plane_threshold = 0.01
    request.filter_option.cluster_threshold = 0.05
    request.filter_option.leaf_size = 0.005
    request.area.min_x = 0.1
    request.area.max_x = 1.0
    request.area.min_y = 0.1
    request.area.max_y = 1.0
    request.area.min_z = -0.1 # Don't filter out the floor
    request.area.max_z = 0.4
    
    try:
      result = self.find_roi_client(request)
      return result.rois 
    except:
      return []
