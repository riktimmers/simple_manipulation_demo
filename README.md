# simple_manipulation_demo
This is a simple manipulation demo, using ROS, MoveIt, PCL and Tensorflow to sort the colored objects into the same colored bin. 

Build and run instructions: 
```
# TODO
```

## Franka Description
The franka_description package is taken from https://github.com/frankaemika/franka_ros, but can be used without having to install the libfranka library. 

## Gazebo-pkgs 
The gazebo-pkgs package is taken from https://github.com/JenniferBuehler/gazebo-pkgs, it is used for the Grasp plugin, which allows better grasping in Gazebo.

## Moveit Planning
This package contains the Moveit planning parts, it offers services for the arm to move, and grasp objects. 

## Object Recognition
This package contains the object recognition parts, It has scripts for data gathering, training and running the object recognition server. 

## Panda Moveit Config 
This package contains the Moveit configuration for the panda arm, including the Octomap of the overhead camera. 

## Panda Simulation
The original package is from https://github.com/erdalpekel/panda_simulation, this package of the Panda is used and modified such that it doesn't require libfranka to be installed.

## Perception
This package contains the perception parts, it can find the bounding box of an object, and the Region Of Interest of the object in the 2d image of the camera. 
