# Package
The original package is from https://github.com/erdalpekel/panda_simulation, this package of the Panda is used and modified such that it doesn't require libfranka to be installed. This Package also works under Ubuntu 20.04 running ROS Noetic. 

# panda_simulation
![Panda in Gazebo](assets/panda-in-gazebo.png?raw=true "Panda in Gazebo")

This package was written for ROS melodic running under Ubuntu 18.04. 

Currently it includes a controller parameter config file and a launch file to launch the [Gazebo](http://gazebosim.org) simulation environment and the Panda robot from FRANKA EMIKA in it with the necessary controllers.

Depending on your operating systems language you might need to export the numeric type so that rviz can read the floating point numbers in the robot model correctly:

```
export LC_NUMERIC="en_US.UTF-8"
```
Otherwise, the robot will appear in rviz in a collapsed state.