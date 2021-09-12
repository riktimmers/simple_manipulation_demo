#include <ros/ros.h>
#include "moveit_planning/moveit_planning.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_planning_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4); // MoveIt requires an AsyncSpinner
  spinner.start();

  MoveitPlanning moveit_planning(node_handle);

  ros::waitForShutdown();

}