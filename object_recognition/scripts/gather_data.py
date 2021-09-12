import rospy 
from sensor_msgs.msg import Image
from roi import ROI
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys
import os
import rospkg
import csv

## Script for gathering data, using an RGB-D camera. 
## Using the ROI class to get region of interest of the object
## Saves the full image containing the object, and saves the crop values in a cvs file. 
## Usage: python3 gather_data.py object_name 
## Will save the data in the object_recognition/data/training/object_name path

class ImageCropper(object):

  # Initializations
  def __init__(self):
    self.data = []
    self.cv_bridge = CvBridge()
    self.roi = ROI()
    self.count = 0
    self.data = []
    self.labels = ["redbox", "bluebox", "greenbox"] # Labels of the possible objects

    if len(sys.argv) != 2: # Check if the current name is given for data gathering
      print("Give name of object")
      exit()

    self.current_label = str(sys.argv[1])
    rospack = rospkg.RosPack()
    self.path = rospack.get_path("object_recognition")
    self.path_training = os.path.join(os.path.join(rospack.get_path("object_recognition"), "data/training/"), self.current_label)

    # If path doesn't exist, create it 
    if not os.path.exists(self.path_training):
      os.makedirs(self.path_training)

    self.label = [0] * len(self.labels)
    # Use a timer callback to request a new image 
    self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

  # Timer callback function
  def timer_callback(self, event):
    # Get the sensor_msgs Image
    image_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    # Convert sensor_msgs Image to openCv image
    image = self.cv_bridge.imgmsg_to_cv2(image_msg)
    original_image = image.copy() # Create a copy of the original (used for saving)

    rois = self.roi.get_rois() # Get the region of interests (roi)

    if len(rois) != 1: # If multiple or none rois are found, don't do anything
      return

    for roi in rois:
      left = roi.left
      right = roi.right 
      top = roi.top
      bottom = roi.bottom 
      # Draw the roi on the image
      cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 1)

    # Show the image with the ROI drawn on it 
    cv2.imshow("Image", image)
    key = cv2.waitKey(1) & 0xFF

    # If 'q' is pressed, save the data and quit
    if key == ord('q'):

      with open(os.path.join(self.path_training, "data.csv"), "wb") as f:
        writer = csv.writer(f)
        writer.writerows(self.data)

      rospy.signal_shutdown("done")
      return

    # Save the original image (with no drawing on it) in the correct label folder and append the roi data
    if key == ord(' '):
      label = [0] * len(self.labels)
      index = self.labels.index(self.current_label)
      label[index] = 1

      image_path = os.path.join(os.path.join("/data/training/", self.current_label), str(self.count) + ".jpg")
      cv2.imwrite(os.path.join(self.path_training, str(self.count) + ".jpg"), original_image)
      self.data.append([image_path, left, right, top, bottom]) 
      self.count += 1
      print(self.count)

if __name__ == "__main__":
  rospy.init_node("gather_data")
  image_cropper = ImageCropper()
  rospy.spin()
