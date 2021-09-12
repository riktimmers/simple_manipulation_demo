import rospy 
from tensorflow.keras import models
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf
import rospkg 
import os
import cv2
import numpy as np 
from object_recognition.msg import ObjectRecognitionAction, ObjectRecognitionResult, Object
import actionlib
from roi import ROI

# Turn on memory growth, don't need to allocate all GPU memory
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
   try:
     # Currently, memory growth needs to be the same across GPUs
     for gpu in gpus:
       tf.config.experimental.set_memory_growth(gpu, True)
     logical_gpus = tf.config.experimental.list_logical_devices('GPU')
     print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
   except RuntimeError as e:
     # Memory growth must be set before GPUs have been initialized
     print(e)

# Action server for object recognition
class ObjectRecognitionServer(object):
  
  # Load the keras model and start the action server
  def __init__(self):
    rospack = rospkg.RosPack()
    path = rospack.get_path("object_recognition")
    model_path = os.path.join(path, "data/training/model.h5")
    self.model = models.load_model(model_path) # Load the model
    self.image_size = 32 # Defined by the training step
    self.labels = ["redbox", "greenbox", "bluebox"]

    # Initialize model by inputting a fake image (first time seems to be slowest)
    fake_image = np.zeros((self.image_size, self.image_size, 3))
    pred = self.model.predict(np.asarray([fake_image]))[0]

    # Publisher for showing the ROI and Label in the image
    self.image_publisher = rospy.Publisher("/object_recognition/image", Image, queue_size=1)

    self.roi = ROI()
    self.cv_bridge = CvBridge()
    self.object_recognition_server = actionlib.SimpleActionServer("/object_recognition", ObjectRecognitionAction, self.object_recognition_callback, auto_start = False)
    self.object_recognition_server.start()

  # Action server callback function
  def object_recognition_callback(self, goal_msg):
    # Get the latest sensor_msgs Image
    image_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    # Conver sensor_msg Image to OpenCv Image
    image = self.cv_bridge.imgmsg_to_cv2(image_msg) 

    # Get the region of interests (roi) 
    rois = self.roi.get_rois()

    if len(rois) == 0: # If no rois where found, abort action server
      self.object_recognition_server.set_aborted()      
      return

    data = []
    result = ObjectRecognitionResult()    
    image_segmented = image.copy() # Image segmented is for publishing 
    roi_data = []

    for roi in rois:
      roi_image = image[roi.top: roi.bottom, roi.left: roi.right] # Get the roi image
      roi_data.append([(roi.left, roi.top), (roi.right, roi.bottom)])
      resized_image = cv2.resize(roi_image, (self.image_size, self.image_size)) # Resize the roi image to fit the CNN
      normalized_image = resized_image / 255. # Normalize between 0.0-1.0
      data.append(normalized_image)
      obj = Object() 
      obj.x = roi.x
      obj.y = roi.y
      obj.z = roi.z
      result.objects.append(obj)
    
    # Classifiy the rois 
    predictions = self.model.predict(np.asarray(data))

    # Draw the roi and label, used for debuggin/visualizing 
    for index, prediction in enumerate(predictions):
      result.objects[index].label = self.labels[np.argmax(prediction)]
      cv2.rectangle(image_segmented, roi_data[index][0], roi_data[index][1], (0, 255, 0))
      cv2.putText(image_segmented, result.objects[index].label, roi_data[index][0], cv2.FONT_HERSHEY_COMPLEX,  
                   1.0, (0, 255, 0)) 

    # Convert OpenCv image to sensor_msgs Image
    publish_msg = self.cv_bridge.cv2_to_imgmsg(image_segmented)
    publish_msg.header.frame_id = "camera_link"
    self.image_publisher.publish(publish_msg)

    # Return results and set action server to success
    self.object_recognition_server.set_succeeded(result)

if __name__ == "__main__":
  rospy.init_node("object_recognition")
  object_recognition_server = ObjectRecognitionServer()
  rospy.spin()
