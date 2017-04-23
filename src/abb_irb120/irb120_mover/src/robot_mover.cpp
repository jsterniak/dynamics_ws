#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <std_msgs/Float64MultiArray.h>

geometry_msgs::Pose ee_target_pose;
std::vector<double> ee_target_joints(6);

unsigned char flag = 0;

int type = 0;

void EEPoseCallBack(const std_msgs::Float64MultiArray msg)
{

    if (msg.data[7] == 0)
    {
        ee_target_pose.orientation.w = msg.data[0];
        ee_target_pose.orientation.x = msg.data[1];
        ee_target_pose.orientation.y = msg.data[2];
        ee_target_pose.orientation.z = msg.data[3];
        ee_target_pose.position.x = msg.data[4];
        ee_target_pose.position.y = msg.data[5];
        ee_target_pose.position.z = msg.data[6];
        flag = 1;
        type = 0;
        std::cout << "Robot Mover Receiving: ";
        std::cout << ee_target_pose << std::endl;
    }
    else if (msg.data[7] == 1){
        ee_target_joints[0] = msg.data[0];
        ee_target_joints[1] = msg.data[1];
        ee_target_joints[2] = msg.data[2];
        ee_target_joints[3] = msg.data[3];
        ee_target_joints[4] = msg.data[4];
        ee_target_joints[5] = msg.data[5];
        type = 1;
        flag = 2;
        std::cout << "Robot Mover Receiving: ";
        std::cout << ee_target_joints[0] << ", " 
            << ee_target_joints[1] << ", " 
            << ee_target_joints[2] << ", " 
            << ee_target_joints[3] << ", " 
            << ee_target_joints[4] << ", " 
            << ee_target_joints[5] << ", " << std::endl;
    }


}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "irb120_robot_mover");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/irb120/ee_pose", 1000, EEPoseCallBack);  

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("manipulator");


    group.setPlanningTime(5.0);

    while (ros::ok()){

        if(flag == 1 && type == 0){
            flag = 0;
            ROS_INFO("set new pose");
            group.setPoseTarget(ee_target_pose);

            moveit::planning_interface::MoveGroup::Plan my_plan;
            bool success = group.plan(my_plan);

            // Uncomment below line when working with a real robot
            group.move();
            sleep(5.0);
            //ROS_INFO("Going to arbitrary position in 2 seconds");
        }

        else if (flag == 2 && type == 1) {
            flag = 0;
            ROS_INFO("set new pose");
            group.setJointValueTarget(ee_target_joints);

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
