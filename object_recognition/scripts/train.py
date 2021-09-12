from tensorflow.keras import models, layers, losses
import tensorflow as tf 
import os 
import numpy as np 
import rospkg
import csv
import cv2

# Don't allocate all memory, only use what is needed
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

rospack = rospkg.RosPack()
name_labels = ["redbox", "greenbox", "bluebox"] # The labels
ros_path = rospack.get_path("object_recognition")
path = os.path.join(ros_path, "data/training")

IMAGE_SIZE = 32 # The images size
data_x = []
data_y = []

padding = [0, 5, 10, 15, 20, 25] # Different padding for data augmentation
value_factors = [0.8, 0.9, 1.0, 1.1, 1.2] # Differnt scaling factor for the Value, making the images lighter or darker

for labels in name_labels: # For all the labels 
  with open(os.path.join(path, labels) + "/data.csv", "rb") as f:
    reader = csv.reader(f)

    for row in reader: # For each image
      image_path = row[0] # The path to the Image
      # Get the ROI values
      left = int(row[1]) 
      right = int(row[2])
      top = int(row[3])
      bottom = int(row[4])
      image = cv2.imread(ros_path + image_path) # Load the image
      label = [0] * len(name_labels) 
      label[name_labels.index(labels)] = 1 # Set the label vector to 1 for the correct label

      height, width, _ = image.shape

      for pad in padding: # For the different paddings
        # Make sure the paddings doesn't go outside of the image boundaries
        l = left - pad if left - pad > 0 else 0
        r = right + pad if right + pad < width else width - 1
        t = top - pad if top - pad > 0 else 0
        b = bottom + pad if bottom + pad else height - 1

        # Crop the image
        roi_image = image[t:b, l:r] 

        # make the ROI image lighter or darker based on the value factor
        for value_factor in value_factors:
          hsv_image = cv2.cvtColor(roi_image.copy(), cv2.COLOR_BGR2HSV) # conver BGR image to HSV image
          hsv_image[:, :, 2] = hsv_image[:, :, 2] * value_factor # Multiply the Value channel with the value factor
          bgr_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR) # Convert back to BGR image
          resized_image = cv2.resize(bgr_image, (IMAGE_SIZE, IMAGE_SIZE)) # Resize image 
          normalized_image = resized_image / 255.0 # Normalize image between 0.0-1.0

          data_x.append(normalized_image)
          data_y.append(label)

# Convert data to numpy arrays
data_x = np.asarray(data_x)
data_y = np.asarray(data_y)

input_shape = data_x[0].shape # Input shape
nr_of_classes = len(data_y[0]) # Nr of classes for the output layer

EPOCHS = 10 # Nr of epochs to train for
BATCH_SIZE = 25 # Batch size 

# Create a (simple) CNN Architecture (The simple objects in sim don't require a complex network)
model = models.Sequential()
model.add(layers.Conv2D(32, (3,3), activation="relu", input_shape=input_shape))
model.add(layers.MaxPooling2D((2,2,)))
model.add(layers.Conv2D(32, (3,3), activation="relu"))
model.add(layers.Conv2D(64, (3,3), activation="relu"))
model.add(layers.Flatten())
model.add(layers.Dense(64))
model.add(layers.Dense(nr_of_classes, activation="softmax")) # Final layer is a softmax for classification

model.compile(optimizer="adam", loss="categorical_crossentropy", metrics=["accuracy"])
model.fit(data_x, data_y, epochs = EPOCHS, batch_size = BATCH_SIZE, shuffle = True, verbose = 1)

model.save(os.path.join(path, "model.h5"))
