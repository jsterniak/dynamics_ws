#include "irb120_actuator.hpp"

#include <std_msgs/Int32.h>
#include <ros/ros.h>

int robot_state = -1;

void GetRobotState(const std_msgs::Int32 msg)
{
    robot_state = msg.data;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "irb120_actuator");
    ros::NodeHandle n;
    ros::Subscriber robot_state_sub = n.subscribe("/irb120/robot_state", 1000, GetRobotState);      // Subscriber to get the real robot JointAngles

    ros::Publisher robot_state_pub =  n.advertise<std_msgs::Int32>("/irb120/actuation_state_complete", 100);        // Publisher to broadcast the current state of the robot

    //  CIRBActuator t_actuator("/dev/ttyUSB0", 9600U);
    CIRBActuator t_actuator("/dev/ttyACM0", 9600U);

    ros::Rate t_loop_rate(1); // 1 Hz

    while (ros::ok())
    {

        switch (robot_state) {
            case 5: //PickSyringe
                // check for parameter update and send
                std::cout << "PickSyringe sending 1" << std::endl;
                t_actuator.send_byte(false, 1);
                // sleep(1.0);
                // t_actuator.send_byte(false, 0);

                break;

            case 13: // PickSuction
                // check for parameter update and send
                std::cout << "PickSuction sending 1" << std::endl;
                t_actuator.send_byte(false, 1);
                // sleep(1.0);
                // t_actuator.send_byte(false, 0);

                break;

            case 21: // Pick HOt air pencil
                // check for parameter update and send
                t_actuator.send_byte(false, 1);
                std::cout << "PickHotAirPencil sending 1" << std::endl;
                // sleep(1.0);
                // t_actuator.send_byte(false, 0);

                break;

            case 11: // DropSyringe
                // check for parameter update and send
                t_actuator.send_byte(false, 2);
                std::cout << "DropSyringe sending 2" << std::endl;
                // sleep(1.0);
                // t_actuator.send_byte(false, 0);

                break;

            case 19: //DropSuction
                // check for parameter update and send
                std::cout << "DropSuction sending 2" << std::endl;
                t_actuator.send_byte(false, 2);
                // sleep(1.0);
                // t_actuator.send_byte(false, 0);

                break;

            case 25: //DropHotAirPencil
                // check for parameter update and send
                t_actuator.send_byte(false, 2);
                std::cout << "DropHotAirPencil sending 2" << std::endl;
                // sleep(1.0);
                // t_actuator.send_byte(false, 0);

                break;

            case 15: // PickSOIC
                // check for parameter update and send
                t_actuator.send_byte(false, 4);
                std::cout << "Pick SOIC sending 4" << std::endl;
                break;

            case 17: // place SOIC
                // check for parameter update and send
                t_actuator.send_byte(false, 0);
                std::cout << "PlaceSOIC sending 1" << std::endl;
                break;

            case 7: // ApplySolderPaste
                //Ethan
                break;
            case 26:
                // Return Home
                t_actuator.send_byte(false, 0);
                std::cout << "ReturnHome sending 0" << std::endl;
            break;

            case 23:
                sleep(10);
            break;
        }

        std_msgs::Int32 state_msg;
        state_msg.data = robot_state;
        //f_pub.publish(state_msg);

        robot_state_pub.publish(state_msg);     


        t_loop_rate.sleep();
        ros::spinOnce();
    }

}
