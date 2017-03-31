#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include "MoveRobot.h"

#define ERROR_THRESHOLD 0.0

//void EEPositionFeedBackCB (const sensor_msgs::JointState msg);

// For all movement state, if the error between the current position and desired position is within a set threshold, move to next state

// Message type of publishing the trajectory to moveit would be a vector of structs containing:
// vector of 6 doubles (representing a list of joint angle position on the path)
// A double for time

// For pickup and drop actions, apply actions for a constant time then move to next state

// Also publishing the current state for other nodes at 50ms

// State 0:
// Initialize and Move to PCB

// State 1:
// At PCB, detect PCB position using tf with the computer vision node, cv publish transform broadcaster, takes input as a raw image stream
// Have the transformation from PCB available using can_tranform(). If yes, move to state 3

// State 2:
// Move to SOIC, move to state 4

// State 3:
// At SOIC, detect SOIC using tf with the computer vision node
// Have the transformation from SOIC available using can_transform(). If yes, move to state 5

// State 4:
// Move to syringe position
// Location of syringe will be hardcoded

// State 5:
// pick up the syringe 
// Move to state 6

// State 6:
// Move to PCB
// Move to state 7

// State 7:
// Apply solder paste
// Move to state 8

// State 8:
// Move to solder paste area
// Move to state 9

// State 9:
// Release solder paste tool
// Move to state 10

// State 10:
// Move to suction cup
// Suction cup location hardcoded
// Move to state 11

// State 11:
// Pick up suction cup
// Move to state 12

// State 12:
// Move to SOIC
// Move to state 13

// State 13
// Pick up SOIC
// Move to state 14

// State 14
// Move to PCB
// Move to state 15

// State 15
// Place SOIC
// Move to state 16

// State 16
// Move to suction cup holder
// Move to state 17

// State 17
// Drop suction cup holder
// Move to state 18

// state 18
// Move to hot air pencil
// Move to state 19

// state 19
// Pick up hot air pencil
// Move to state 20

// state 20
// Move to PCB
// move to state 21

// state 21
// Apply hot air
// move to state 22

// state 22
// move to hot air pencil holder
// move to state 23

// state 23
// drop hot air pencil
// move to 24

// state 24
// return home

// state 25
// terminate


enum RobotState {
    Initialization = 0, 
    DetectPCB = 1, 
    Move2DetectSOIC = 2,
    DetectSOIC = 3,
    Move2PickSyringe = 4,
    PickSyringe = 5,
    Move2ReleaseSolderPaste = 6,
    ApplySolderPaste = 7,
    Move2DropSyringe = 8,
    DropSyringe = 9,
    Move2PickSuction = 10,
    PickSuction = 11,
    Move2PickSOIC = 12,
    PickSOIC = 13,
    Move2PlaceSOIC = 14,
    PlaceSOIC = 15,
    Move2DropSuction = 16,
    DropSuction = 17,
    Move2PickHotAirPencil = 18,
    PickHotAirPencil = 19,
    Move2SolderPCB = 20,
    ApplyHotAir = 21,
    Move2DropHotAirPencil = 22,
    DropHotAirPencil = 23,
    ReturnHome = 24
};

int current_robot_state = 0;

MatrixXd JointAngles(1,6);          // The robot current joint angles
VectorXd ee_current_position(3);    // The robot real position of the end-effector calculated from the JointAngles

void EEPositionFeedBackCB (const sensor_msgs::JointState msg)
{

    JointAngles(0,0)=msg.position[0];
    JointAngles(0,1)=msg.position[1];
    JointAngles(0,2)=msg.position[2];
    JointAngles(0,3)=msg.position[3];
    JointAngles(0,4)=msg.position[4];
    JointAngles(0,5)=msg.position[5];

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "irb120_state_machine");
    ros::NodeHandle n;
    ros::Publisher robot_state_pub = n.advertise<std_msgs::String>("robot_state", 1000);        // Publisher to broadcast the current state of the robot
    ros::Publisher ee_position_pub = n.advertise<geometry_msgs::Point>("/irb120/ee_pos", 1000); // Publisher to broadcast the position the EE needs to be at

    ros::Subscriber ee_pos_sub  = n.subscribe("/joint_states", 100, EEPositionFeedBackCB);      // Subscriber to get the real robot JointAngles

    ros::Rate loop_rate(2); // Publishing at 50 ms = 2Hz


    // Initializing D-H parameters of the robot
    VectorXd theta(6);
    VectorXd d(6);
    VectorXd a(6);
    VectorXd alpha(6);

    d(0) = 290; d(1)=0; d(2) = 0; d(3) = 302; d(4) = 0; d(5)= 72;
    a(0) = 0; a(1) = 270; a(2) = 70; a(3) = 0; a(4) = 0; a(5) = 0;
    alpha(0) = 90 * (M_PI/180.0); alpha(1) = 0* (M_PI/180.0); alpha(2) = 90* (M_PI/180.0);
    alpha(3) = -90* (M_PI/180.0); alpha(4) = 90* (M_PI/180.0); alpha(5)= 0* (M_PI/180.0);

    JointAngles(0,0) = 0.0;
    JointAngles(0,1) = 0.0;
    JointAngles(0,2) = 0.0;
    JointAngles(0,3) = 0.0;
    JointAngles(0,4) = 0.0;
    JointAngles(0,5) = 0.0;

    // MoveBot object to calculate Kinematics
    MoveRobot move_bot(theta, d, a, alpha);

    // The homogeneous transformation matrix H from base to tip
    MatrixXd H;

    while (ros::ok()){

        // Define vectors that represent the position of the PCB and error
        VectorXd desired_position(3);
        VectorXd ee_error(3);

        std_msgs::String state_msg;
        geometry_msgs::Point desired_position_msg;

        switch(current_robot_state) {
            case Initialization: 
                // This state initilizes the robot position above the PCB, a hardcoded position where the camera can easily detect the PCB

                // Calculate the homogeneous matrix to get the current end-effector position

                H = move_bot.getHomogeneous(JointAngles, d, a, alpha);
                ee_current_position(0) = H(0, 3);
                ee_current_position(1) = H(1, 3);
                ee_current_position(2) = H(2, 3);

                // Set the position of the PCB
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;


                // use ee_position_pub to publish PCB position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg); 

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = DetectPCB;
                }

                // Publish the robot current state
                state_msg.data = "Initialization";
                robot_state_pub.publish(state_msg);

                break;

            case DetectPCB:

                break;

            case Move2DetectSOIC:

                break;

            case DetectSOIC:

                break;

            case Move2PickSyringe:

                break;

            case PickSyringe:

                break;

            case Move2ReleaseSolderPaste:

                break;

            case ApplySolderPaste:

                break;

            case Move2DropSyringe:

                break;

            case DropSyringe:

                break;

            case Move2PickSuction:

                break;

            case PickSuction:

                break;

            case Move2PickSOIC:

                break;

            case PickSOIC:

                break;

            case Move2PlaceSOIC:

                break;

            case PlaceSOIC:

                break;

            case Move2DropSuction:

                break;


            case DropSuction:

                break;

            case Move2PickHotAirPencil:

                break;

            case PickHotAirPencil:

                break;

            case Move2SolderPCB:

                break;

            case ApplyHotAir:

                break;

            case Move2DropHotAirPencil:

                break;

            case DropHotAirPencil:

                break;

            case ReturnHome:

                break;


        }
        ros::spinOnce();
        loop_rate.sleep();    
    }

    // Computer vision to detect all positions

    // Pick up syringe

    // Move to PCB and apply solder paste

    // Switch tool too suction cup

    // Move to and Pick up SOIC

    // Move to PCB and place SOIC

    // Switch tool to hot air pencil

    // Move to PCB

    // Flow solder

    // Put tool away and return home

    return 0;
}
