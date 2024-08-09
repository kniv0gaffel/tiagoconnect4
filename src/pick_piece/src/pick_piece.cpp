// ROS headers
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <map>
#include <string>
#include <vector>

geometry_msgs::PoseStamped goal_pose;
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  goal_pose = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "plan_arm_torso_ik");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/aruco_piece/pick", 1, &goalPoseCallback);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  // select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  // set intermediate waypoint
  group_arm_torso.setNamedTarget("hover");
  geometry_msgs::PoseStamped hover_pose = goal_pose;
  hover_pose.pose.position.z += 0.2;
  // choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(hover_pose);

  ROS_INFO_STREAM("Planning to move " << group_arm_torso.getEndEffectorLink()
                                      << " to a target pose expressed in "
                                      << group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  moveit::planning_interface::MoveItErrorCode success =
      group_arm_torso.plan(my_plan);

   if (success != 1)
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm_torso.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  // wait some milliseconds and go to end position and grab the piece
  ros::Duration(0.5).sleep();
  group_arm_torso.setNamedTarget("end");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " << group_arm_torso.getEndEffectorLink()
                                      << " to a target pose expressed in "
                                      << group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  // set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  success = group_arm_torso.plan(my_plan);

  if (success != 1)
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  start = ros::Time::now();

  group_arm_torso.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  spinner.stop();

  return EXIT_SUCCESS;
}
