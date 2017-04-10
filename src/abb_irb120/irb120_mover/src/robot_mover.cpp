#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>


void EEPoseCallback(const geometry_msgs::Pose msg)
{
		
	geometry_msgs::Pose ee_target_pose
	ee_target_pose.orientation.w = 1.0;
	ee_target_pose.orientation.x = 8.2768e-06;
    ee_target_pose.orientation.y = 2.0115e-05;
    ee_target_pose.orientation.z = 2.0115e-05;
    ee_target_pose.position.x = msg.x;
    ee_target_pose.position.y = msg.y;
    ee_target_pose.position.z = msg.z;

    group.setPoseTarget(ee_target_pose);
	
	moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    // Uncomment below line when working with a real robot
    //  group.move();
	
	ROS_INFO("Moving to new pose");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "irb120_robot_mover");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/irb120/ee_pos", 1000, chatterCallback);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("manipulator");
	group.setPlanningTime(1.0);

	ROS_INFO("Going to arbitrary position in 2 seconds");
	
	ros::spin();

	return 0;
}
