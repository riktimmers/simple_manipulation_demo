#ifndef _H_MOVEIT_PLANNING__
#define _H_MOVEIT_PLANNING__

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <map>
#include <vector>
#include <string>
#include <tf2/utils.h>
#include <moveit_planning/MoveToPose.h>
#include <moveit_planning/MoveToJointTarget.h>
#include <moveit_planning/AddBox.h>
#include <moveit_planning/RemoveBox.h>
#include <moveit_planning/Grasp.h>
#include <perception/BoundingBox.h>
#include <std_srvs/Empty.h>

class MoveitPlanning {

  ros::NodeHandle node_handle_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::MoveGroupInterface gripper_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  ros::ServiceServer move_to_pose_server_;
  ros::ServiceServer move_to_joint_target_server_;
  ros::ServiceServer add_box_server_;
  ros::ServiceServer remove_box_server_;
  ros::ServiceServer grasp_server_;
  ros::ServiceServer close_fingers_server_;
  ros::ServiceServer open_fingers_server_;
  ros::ServiceClient clear_octomap_;

  public: 
    MoveitPlanning(ros::NodeHandle &node_handle);
    bool moveToPose(moveit_planning::MoveToPoseRequest &req, moveit_planning::MoveToPoseResponse &res);
    bool moveToJointTarget(moveit_planning::MoveToJointTargetRequest &req, moveit_planning::MoveToJointTargetResponse &res);
    bool addBox(moveit_planning::AddBoxRequest &req, moveit_planning::AddBoxResponse &res);
    bool removeBox(moveit_planning::RemoveBoxRequest &req, moveit_planning::RemoveBoxResponse &res);
    bool grasp(moveit_planning::GraspRequest &req, moveit_planning::GraspResponse &res);
    bool closeFingers(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool openFingers(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
  
  private:
    void moveToStartPose();
    moveit_msgs::CollisionObject createBox(float length, float width, float height);
    void addToPlanningScene(perception::BoundingBox bounding_box, std::string name = "object");
    void removeFromPlanningScene(std::string frame_id, std::string id);

    bool graspObject(perception::BoundingBox &bounding_box);
    void generateGrasps(perception::BoundingBox &bounding_box, std::vector<moveit_msgs::Grasp> &grasps);
    void preGraspPosture(trajectory_msgs::JointTrajectory &posture);
    void postGraspPosture(trajectory_msgs::JointTrajectory &posture, float width);
    void generatePosture(trajectory_msgs::JointTrajectory &posture);
};

#endif