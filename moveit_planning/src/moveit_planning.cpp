#include "moveit_planning/moveit_planning.h"

//! Constructor, initalized the planning groups, and the service servers. Finally move the Arm to an init position.
/** \param node_handle is a ros::NodeHandle. 
 */
MoveitPlanning::MoveitPlanning(ros::NodeHandle &node_handle) : 
      move_group_("arm"), 
      gripper_("gripper"),
      node_handle_(node_handle) {
  move_to_pose_server_ = node_handle_.advertiseService("/move_to_pose", &MoveitPlanning::moveToPose, this);
  move_to_joint_target_server_ = node_handle_.advertiseService("/move_to_joint_target", &MoveitPlanning::moveToJointTarget, this);
  add_box_server_ = node_handle_.advertiseService("/add_box", &MoveitPlanning::addBox, this);
  remove_box_server_ = node_handle_.advertiseService("/remove_box", &MoveitPlanning::removeBox, this);
  grasp_server_ = node_handle_.advertiseService("/grasp", &MoveitPlanning::grasp, this);
  open_fingers_server_ = node_handle_.advertiseService("/open_fingers", &MoveitPlanning::openFingers, this);
  close_fingers_server_ = node_handle_.advertiseService("/close_fingers", &MoveitPlanning::closeFingers, this);

  clear_octomap_ = node_handle_.serviceClient<std_srvs::Empty>("/clear_octomap");

  move_group_.setPlanningTime(60.0); 
  move_group_.setMaxAccelerationScalingFactor(1.0); // Default 0.1 scaling
  move_group_.setMaxVelocityScalingFactor(1.0); // Default 0.1 scaling
  moveToStartPose();
}

//! Function call that sets Joints Target to initialize the robot. 
void MoveitPlanning::moveToStartPose() {
  std::map<std::string, double> joint_positions;
  joint_positions.insert(std::pair<std::string, double>("panda_joint1", 0.0));
  joint_positions.insert(std::pair<std::string, double>("panda_joint2", 0.0));
  joint_positions.insert(std::pair<std::string, double>("panda_joint3", 0.0));
  joint_positions.insert(std::pair<std::string, double>("panda_joint4", -1.57));
  joint_positions.insert(std::pair<std::string, double>("panda_joint5", 0));
  joint_positions.insert(std::pair<std::string, double>("panda_joint6", 1.57));
  joint_positions.insert(std::pair<std::string, double>("panda_joint7", 0));
  

  move_group_.setJointValueTarget(joint_positions); // Set target positions
  moveit_msgs::MoveItErrorCodes error = move_group_.move(); // Start planning and moving if a plan is found

  if (error.val == error.SUCCESS) {
    ROS_INFO_STREAM("Success");
  } else {
    ROS_INFO_STREAM("FAILED");
  }
}

//! Function call to create a collision object (a Box)
/** \param length a float that sets the length of the box in meters.
 *  \param width a float that sets the width of the box in meters.
 *  \param height a float that sets the height of the box in meters.
 *  \return moveit_msgs::CollisionObject, a collision message that is initalized with the given dimensions.
 */ 
moveit_msgs::CollisionObject MoveitPlanning::createBox(float length, float width, float height) {
  moveit_msgs::CollisionObject object;
  
  object.primitives.resize(1);
  object.primitives.at(0).type = object.primitives.at(0).BOX;
  object.primitives.at(0).dimensions.resize(3);
  object.primitives.at(0).dimensions.at(0) = length;
  object.primitives.at(0).dimensions.at(1) = width;
  object.primitives.at(0).dimensions.at(2) = height;
  object.operation = object.ADD;

  return object;
}

//! Function call to add a bounding box as a collision model to the planning scene.
/** \param bounding_box is a perception::BoundingBox message, that contains the center position, yaw rotation, and the dimensions of the box.
 *  \param name is std::string that give the object an id. 
 */
void MoveitPlanning::addToPlanningScene(perception::BoundingBox bounding_box, std::string name) {
  moveit_msgs::CollisionObject object;
  float scaling = 1.1; // Make the collision object a little bit larger in case of sensor noise.
  object = createBox(bounding_box.length * scaling, bounding_box.width * scaling, bounding_box.height * scaling);
  object.id = name;
  object.header.frame_id = "panda_link0"; // Hardcoded, always use the panda_link0 as frame of reference.

  object.primitive_poses.resize(1);
  object.primitive_poses.at(0).position.x = bounding_box.x;
  object.primitive_poses.at(0).position.y = bounding_box.y;
  object.primitive_poses.at(0).position.z = bounding_box.z;
  tf2::Quaternion quaternion;
  quaternion.setEuler(0, 0, bounding_box.yaw); 
  object.primitive_poses.at(0).orientation = tf2::toMsg(quaternion);

  // Add collision model to the planning scene.
  planning_scene_interface_.applyCollisionObject(object);
}

//! Function call to remove a collision model from the planning scene.
/** \param frame_id is a std::string of the frame_id that the object is attached to.
 *  \param id is a std::String is the name of the object to remove.
 */
void MoveitPlanning::removeFromPlanningScene(std::string frame_id, std::string id) {
  moveit_msgs::CollisionObject remove_object;
  remove_object.header.frame_id = frame_id;
  remove_object.id = id;
  remove_object.operation = remove_object.REMOVE; 

  planning_scene_interface_.applyCollisionObject(remove_object);
}
