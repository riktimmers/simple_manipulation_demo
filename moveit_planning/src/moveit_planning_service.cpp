#include "moveit_planning/moveit_planning.h"

//! Service callback to move the arm given a specific pose of the End Effector.
/** \param req is moveit_planning::MoveToPoseRequest, which contains a PoseStamped message for the pose of the End Effector.
 *  \param res is moveit_planning::MoveToPoseResonse which will return an error value. 
 *  \return bool, return true if the planning and executation was succesfull, else return false.
 */
bool MoveitPlanning::moveToPose(moveit_planning::MoveToPoseRequest &req, moveit_planning::MoveToPoseResponse &res) {
  move_group_.setPoseTarget(req.pose); // A PoseStamped message for the pose of the End Effector.

  moveit_msgs::MoveItErrorCodes error = move_group_.move(); // Plan and move to the target pose.

  if (error.val == error.SUCCESS) {
    ROS_INFO_STREAM("Move succesfull");
    res.success = true;
    return true;
  } else {
    ROS_INFO_STREAM("Move failed");
    res.success = false;
    return false;
  }
}

//! Service callback to move the arm given the spefici joint targets. 
/** \param req is a moveit_planning::MoveToJointTargetRequest which containts joint target values.
 *  \param res is a moveit_planning::MoveToJointTargetResponse which will return an error value.
 *  \return bool, return true if the planning and execution was successfull, else return false.
 */
bool MoveitPlanning::moveToJointTarget(moveit_planning::MoveToJointTargetRequest &req, moveit_planning::MoveToJointTargetResponse &res) {
  std::map<std::string, double> joint_positions; // Create map containing the joint names and target values.
  joint_positions.insert(std::pair<std::string, double>("panda_joint1", req.joint1));
  joint_positions.insert(std::pair<std::string, double>("panda_joint2", req.joint2));
  joint_positions.insert(std::pair<std::string, double>("panda_joint3", req.joint3));
  joint_positions.insert(std::pair<std::string, double>("panda_joint4", req.joint4));
  joint_positions.insert(std::pair<std::string, double>("panda_joint5", req.joint5));
  joint_positions.insert(std::pair<std::string, double>("panda_joint6", req.joint6));
  joint_positions.insert(std::pair<std::string, double>("panda_joint7", req.joint7));
  
  move_group_.setJointValueTarget(joint_positions);
  moveit_msgs::MoveItErrorCodes error = move_group_.move(); // Plan and move to the target joint positions.

  if (error.val == error.SUCCESS) {
    res.success = true;
    return true;
  }
  
  res.success = false;
  return false;
}

//! Service call to add a collision object to the planning scene
/** \param req is a moveit_planning::AddBoxRequest which contains the position, orientation and dimensions of the object.
 *  \param res is a moveit_planning::AddBoxResponse is an Empty message and not used.
 *  \return bool, always return true.
 */
bool MoveitPlanning::addBox(moveit_planning::AddBoxRequest &req, moveit_planning::AddBoxResponse &res) {
  moveit_msgs::CollisionObject object;
  float scaling = 1.1; // scaling factor to 
  object = createBox(req.length * scaling, req.width * scaling,  req.height * scaling);
  object.id = req.name;
  object.header.frame_id = req.frame_id;

  object.primitive_poses.resize(1);
  object.primitive_poses.at(0).position.x = req.x;
  object.primitive_poses.at(0).position.y = req.y;
  object.primitive_poses.at(0).position.z = req.z;
  tf2::Quaternion quaternion;
  quaternion.setEuler(req.pitch, req.roll, req.yaw); 
  object.primitive_poses.at(0).orientation = tf2::toMsg(quaternion);

  planning_scene_interface_.applyCollisionObject(object);
  return true;
}

//! Service call for remove a collision object given its frame_id and id.
/** \param req is a moveit_planning::RemoveBoxRequest, which contains the frame_id and the id of the object to remove from the planning scene.
 *  \param res is a moveit_planning::RemoveBoxResponse is an Empty message and not used.
 *  \return bool, always returns true.
 */ 
bool MoveitPlanning::removeBox(moveit_planning::RemoveBoxRequest &req, moveit_planning::RemoveBoxResponse &res) {
  removeFromPlanningScene(req.frame_id, req.id);
  return true;
}

//! Service call for grasping an object.
/** \param req is a moveit_planning::GraspRequest containing the bounding box that needs to be grasped.
 *  \param res is a moveit_planning::GraspResponse is an Empty message and not used.
 *  \return bool, returns true if the planning and execution of the grasp was successfull, else return false.
 */
bool MoveitPlanning::grasp(moveit_planning::GraspRequest &req, moveit_planning::GraspResponse &res) {
  
  if (!graspObject(req.bounding_box)) {
    ROS_INFO_STREAM("Grasp failed, returning false");
    return false;
  }

  return true;
}

//! Service call for opening the fingers 
/** \param req is an std_srvs::EmptyRequest 
 *  \param res is an std_srvs::EmptyResponse
 *  \return bool, will return true if the fingers where able to open, else return false.
 */
bool MoveitPlanning::openFingers(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res) {
  std::map<std::string, double> finger_positions; // Map containing the finger joint names and their position.
  finger_positions.insert(std::pair<std::string, double>("panda_finger_joint1", 0.04));
  finger_positions.insert(std::pair<std::string, double>("panda_finger_joint2", 0.04));
  gripper_.setJointValueTarget(finger_positions);
  moveit_msgs::MoveItErrorCodes error = gripper_.move(); // Move the fingers and return error code

  ros::Rate rate(ros::Duration(2.0));
  rate.sleep();

  // If the fingers have successfully openend, remove possibly attached collision objects from that arm and the scene.
  if (error.val == error.SUCCESS) {
    std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects;
    attached_objects = planning_scene_interface_.getAttachedObjects();

    ROS_INFO_STREAM("Removing objects when opening fingers, amount of objects: " << attached_objects.size());

    // For all attached objects, remove them from the End Effector. 
    for (auto &object: attached_objects) {
      object.second.object.operation = object.second.object.REMOVE;
      planning_scene_interface_.applyAttachedCollisionObject(object.second);
    }

    std::map<std::string, moveit_msgs::CollisionObject> objects;
    objects = planning_scene_interface_.getObjects();

    // Remove all collision models from the planning scene
    for (auto &object: objects) {
      object.second.operation = object.second.REMOVE;
      planning_scene_interface_.applyCollisionObject(object.second);
    }

    return true;
  }
  
  return false;
}

//! Service call for closing the fingers.
/** \param req is a std_srvs::EmptyRequest.
 *  \param res is a std_srvs::EmptyResponse.
 *  \return bool, returns true of the fingers have been able to close, else return false.
 */
bool MoveitPlanning::closeFingers(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res) {
  std::map<std::string, double> finger_positions; // Map containing finger joint names and their position.
  finger_positions.insert(std::pair<std::string, double>("panda_finger_joint1", 0.0));
  finger_positions.insert(std::pair<std::string, double>("panda_finger_joint2", 0.0));

  gripper_.setJointValueTarget(finger_positions);
  moveit_msgs::MoveItErrorCodes error = gripper_.move(); // Move the fingers and return the error code.

  if (error.val == error.SUCCESS) {
    return true;
  }

  return false;
}