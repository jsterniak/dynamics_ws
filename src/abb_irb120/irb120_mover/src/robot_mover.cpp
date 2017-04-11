#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <std_msgs/Float64MultiArray.h>

moveit::planning_interface::MoveGroup group("manipulator");

void EEPoseCallBack(const std_msgs::Float64MultiArray msg)
{

    
    std::vector< double > ee_target_pose;		
	//geometry_msgs::Pose ee_target_pose;

    ee_target_pose.resize(6);  

    ee_target_pose[0] = msg.data[0];
    ee_target_pose[1] = msg.data[1];
    ee_target_pose[2] = msg.data[2];
    ee_target_pose[3] = msg.data[3];
    ee_target_pose[4] = msg.data[4];
    ee_target_pose[5] = msg.data[5];

    group.setJointValueTarget(ee_target_pose);
	
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

	ros::Subscriber sub = n.subscribe("/irb120/joint_values", 1000, EEPoseCallBack);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	group.setPlanningTime(1.0);

	ROS_INFO("Going to arbitrary position in 2 seconds");
	
	ros::spin();

	return 0;
}
