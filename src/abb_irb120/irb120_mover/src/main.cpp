#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irb120_mover_node");
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  group.setPlanningTime(1.0);

  ROS_INFO("Going to arbitrary position in 2 seconds");
  sleep(2.0);

  geometry_msgs::Pose ee_target_pose;
  ee_target_pose.orientation.w = 1.0;
  ee_target_pose.orientation.x = 8.2768e-06;
  ee_target_pose.orientation.y = 2.0115e-05;
  ee_target_pose.orientation.z = 2.0115e-05;
  ee_target_pose.position.x = 0.271271;
  ee_target_pose.position.y = -0.382311;
  ee_target_pose.position.z = 0.379979;
  group.setPoseTarget(ee_target_pose);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  // Uncomment below line when working with a real robot
//  group.move();

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan
  sleep(5.0);

  ROS_INFO("Returning to home position in 2 seconds");
  sleep(2.0);

  ee_target_pose.orientation.w = 1.0;
  ee_target_pose.orientation.x = -8.90995e-06;
  ee_target_pose.orientation.y = 6.98714e-05;
  ee_target_pose.orientation.z = 1.47223e-05;
  ee_target_pose.position.x = 0.374032;
  ee_target_pose.position.y = 1.10126e-05;
  ee_target_pose.position.z = 0.629952;
  group.setPoseTarget(ee_target_pose);

  success = group.plan(my_plan);

  // Uncomment below line when working with a real robot
//  group.move();

  ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan
  sleep(5.0);

  return 0;
}
