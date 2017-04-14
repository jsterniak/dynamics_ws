#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <std_msgs/Float64MultiArray.h>

std::vector<double> ee_target_pose;
unsigned char flag = 0;

void EEPoseCallBack(const std_msgs::Float64MultiArray msg)
{

    ee_target_pose.resize(6);

    ee_target_pose[0] = 1.9;//msg.data[0];
    ee_target_pose[1] = 1.067;//msg.data[1];
    ee_target_pose[2] = 0.082;//msg.data[2];
    ee_target_pose[3] = -0.03051;//msg.data[3];
    ee_target_pose[4] = 0.42777;//msg.data[4];
    ee_target_pose[5] = 2;//msg.data[5];

    flag = 1;
    //std::cout << ee_target_pose[0] << std::endl; 
    //ROS_INFO("Moving to new pose");
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "irb120_robot_mover");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/irb120/joint_values", 1000, EEPoseCallBack);  

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("manipulator");

 
    group.setPlanningTime(5.0);

    while (ros::ok()){

        if(flag == 1){
            flag = 0;
            ROS_INFO("set new pose");
            group.setJointValueTarget(ee_target_pose);

            moveit::planning_interface::MoveGroup::Plan my_plan;
            bool success = group.plan(my_plan);

            // Uncomment below line when working with a real robot
            group.move();
            sleep(5.0);
            //ROS_INFO("Going to arbitrary position in 2 seconds");
        }
        ros::spinOnce();

    }
    return 0;
}
