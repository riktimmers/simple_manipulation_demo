#include "moveit_planning/moveit_planning.h"

//! Function call to grasp an object 
/** \param bounding_box is a perception::BoundingBox of the object to be grasped.
 *  \return bool, returns true if the object was successfully picked up, else return false.
 */
bool MoveitPlanning::graspObject(perception::BoundingBox &bounding_box) {
  std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects;
  std::vector<moveit_msgs::AttachedCollisionObject> objects_to_detach;

  // First make sure there are no objects attached to the End Effector.
  for (auto &object: attached_objects) {
    object.second.object.REMOVE;
    objects_to_detach.push_back(object.second);
  }

  planning_scene_interface_.applyAttachedCollisionObjects(objects_to_detach);
  
  ros::Rate rate(1); 
  addToPlanningScene(bounding_box); // Add the Object to the planning scene as a Collision Object
  std_srvs::EmptyRequest req;
  std_srvs::EmptyResponse res;
  clear_octomap_.call(req, res); // Clear the octomap (sometimes voxels are not removed such that they still collide with the collision model, clearing the octomap fixes this).
  
  rate.sleep(); // Give some time for the Octomap to reset and draw the voxels.
  std::vector<moveit_msgs::Grasp> grasps; 
  generateGrasps(bounding_box, grasps);  // Generate different grasps based on the bounding box information.
  
  moveit_msgs::MoveItErrorCodes error = move_group_.pick("object", grasps); // Plan and execute the grasping of the object.

  if (error.val != error.SUCCESS) {
    ROS_INFO_STREAM("Error value: " << error.val);
    return false;
  }

  return true;
}

//! Function call to generate different grasp. 
/** \param bounding_box is a perception::BoundingBox that contains the information about the position, orientation and dimensions of the object to grasp.
 *  \param grasps is std::vector<moveit_msgs::Grasp> that contains multiple grasps from which the planning pipeline will select a valid grasp. 
 */
void MoveitPlanning::generateGrasps(perception::BoundingBox &bounding_box, std::vector<moveit_msgs::Grasp> &grasps) {
  const float finger_length = 0.04; // The lenght of the fingers

  // From the top of the object move down with finger length to try and find a good grasp. With step size of 0.005m 
  for (auto height = bounding_box.z + bounding_box.height/2; height > bounding_box.z + bounding_box.height/2 - finger_length; height -= 0.005) {
    moveit_msgs::Grasp grasp; 
    grasp.grasp_pose.header.frame_id = "panda_link0"; // Hardcoded, always plan from the panda_link0
    tf2::Quaternion quaternion;
    quaternion.setEuler(M_PI, 0, -bounding_box.yaw); // Set the End Effector's orientation
    grasp.grasp_pose.pose.orientation = tf2::toMsg(quaternion);
    grasp.grasp_pose.pose.position.x = bounding_box.x;
    grasp.grasp_pose.pose.position.y = bounding_box.y;
    grasp.grasp_pose.pose.position.z = height; // Different heights for trying to find a valid grasp.
    grasp.pre_grasp_approach.min_distance = 0.05;
    grasp.pre_grasp_approach.desired_distance = 0.10;
    grasp.pre_grasp_approach.direction.header.frame_id = move_group_.getEndEffectorLink();
    grasp.pre_grasp_approach.direction.vector.z = 1.0f;
    grasp.post_grasp_retreat.min_distance = 0.05;
    grasp.post_grasp_retreat.desired_distance = 0.1;
    grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0";
    grasp.post_grasp_retreat.direction.vector.z = 1.0f;
    
    preGraspPosture(grasp.pre_grasp_posture); // Before grasping open the fingers
    postGraspPosture(grasp.grasp_posture, bounding_box.width - 0.01); // Close the fingers using the width of the object minus 1cm 
    grasps.push_back(grasp);
  }
}

//! Function call to create a pre-grasp posture by opening the fingers
/** \param posture is a trajectory_msgs::JointTrajectory which will contain the joint names and their position.
 */
void MoveitPlanning::preGraspPosture(trajectory_msgs::JointTrajectory &posture) {
  generatePosture(posture); 
  posture.points.at(0).positions.at(0) = 0.04f;
  posture.points.at(0).positions.at(1) = 0.04f;
}

//! Function call to create a post-grasp posture by closing the fingers
/** \param posture is a trajectory_msgs::JointTrajectory which will contain the joint names and their position.
 *  \param width is float that contains the width of the object.
 */
void MoveitPlanning::postGraspPosture(trajectory_msgs::JointTrajectory &posture, float width) {
  generatePosture(posture);
  posture.points.at(0).positions.at(0) = width / 2.0; // Move both fingers to half of the width size.
  posture.points.at(0).positions.at(1) = width / 2.0;
}

//! Function call to generate the Joint Trajectory message.
/** \param posture is a trajectory_msgs::JointTrajectory which will be generated and given the joint names and duration.
 */
void MoveitPlanning::generatePosture(trajectory_msgs::JointTrajectory &posture) {
  posture.joint_names.resize(2); // For naming the two finger joints
  posture.joint_names.at(0) = "panda_finger_joint1";
  posture.joint_names.at(1) = "panda_finger_joint2";
  posture.points.resize(1);
  posture.points.at(0).positions.resize(2);
  posture.points.at(0).time_from_start = ros::Duration(5.0); // Give it 5 seconds for the motion.
}