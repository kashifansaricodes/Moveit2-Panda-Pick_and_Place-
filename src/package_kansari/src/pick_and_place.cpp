#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "package_sjd3333",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("package_sjd3333");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_arm = MoveGroupInterface(node, "panda_arm");
  auto move_group_hand = MoveGroupInterface(node, "hand");

  std::map<std::string, double> joint_values;
  joint_values["panda_finger_joint1"] = 0.0125; 

  move_group_hand.setJointValueTarget(joint_values);


  if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to move the gripper to the specified position!");
        return 1; 
    }

// Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -2.5082286811084487e-05;  // Scalar component (w)
    msg.orientation.x = 1; // Imaginary component along X-axis
    msg.orientation.y = 0.00016352070088032633; // Imaginary component along Y-axis
    msg.orientation.z = 1.1314374205539934e-05; // Imaginary component along Z-axis
    msg.position.x = 0.4082701802253723;
    msg.position.y = -0.5790671110153198;
    msg.position.z = 0.244498148560524;
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_arm]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    if (move_group_arm.execute(plan)) {
      // Set custom joint values to close the gripper
      std::map<std::string, double> joint_values;
      joint_values["panda_finger_joint1"] = 0.00;  
      move_group_hand.setJointValueTarget(joint_values);

      if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to close the gripper!");
      }
    }
  } else {
    RCLCPP_ERROR(logger, "Planning for arm failed!");
  };

// Set a target Pose
  auto const target_pose2 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -0.013705332763493061;  // Scalar component (w)
    msg.orientation.x = 0.9998118281364441; // Imaginary component along X-axis
    msg.orientation.y = 0.00028218948864378035; // Imaginary component along Y-axis
    msg.orientation.z = 0.013727345503866673; // Imaginary component along Z-axis
    msg.position.x = -0.00859931018203497;
    msg.position.y = 0.5924981832504272;
    msg.position.z = 0.16972656548023224;
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose2);

  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_arm]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    if (move_group_arm.execute(plan2)) {
      if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to close the gripper!");
      }
    }
  } else {
    RCLCPP_ERROR(logger, "Planning for arm failed!");
  };

// Set a target Pose
  auto const target_pose3 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 4.523881269591357e-12;  // Scalar component (w)
    msg.orientation.x = 0.9238795042037964; // Imaginary component along X-axis
    msg.orientation.y = 0.3826834261417389; // Imaginary component along Y-axis
    msg.orientation.z = -1.8738066653867236e-12; // Imaginary component along Z-axis
    msg.position.x = 0.08799999952316284;
    msg.position.y = -7.148726055561383e-13;
    msg.position.z = 0.9259999990463257;
    return msg;
  }();
  move_group_arm.setPoseTarget(target_pose3);

  // Create a plan to that target pose
  auto const [success3, plan3] = [&move_group_arm]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_arm.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    if (move_group_arm.execute(plan3)) {
      // Set custom joint values to close the gripper
      std::map<std::string, double> joint_values;
      joint_values["panda_finger_joint1"] = 0.025;  
      move_group_hand.setJointValueTarget(joint_values);

      if (!move_group_hand.move()) {
        RCLCPP_ERROR(logger, "Failed to close the gripper!");
      }
    }
  } else {
    RCLCPP_ERROR(logger, "Planning for arm failed!");
  };

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}