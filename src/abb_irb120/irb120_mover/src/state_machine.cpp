#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include "MoveRobot.h"

#define ERROR_THRESHOLD 0.0


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

int current_robot_state = Initialization;

MatrixXd JointAngles(1,6);          // The robot current joint angles
VectorXd ee_current_position(3);    // The robot real position of the end-effector calculated from the JointAngles

void EEPositionFeedBackCB (const sensor_msgs::JointState msg)
{
    cout << "Real Joint Values [" << msg.position[0];
    cout << ", "<< msg.position[1];
    cout << ", "<< msg.position[2];
    cout << "," << msg.position[3];
    cout << ", "<< msg.position[4];
    cout << ", "<< msg.position[5] << "]" << endl;
    JointAngles(0,0)=msg.position[0];//*(M_PI/180);
    JointAngles(0,1)=msg.position[1];//*(M_PI/180);
    JointAngles(0,2)=msg.position[2];//(M_PI/180);
    JointAngles(0,3)=msg.position[3];//*(M_PI/180);
    JointAngles(0,4)=msg.position[4];//*(M_PI/180);
    JointAngles(0,5)=msg.position[5];//*(M_PI/180);

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "irb120_state_machine");
    ros::NodeHandle n;
    ros::Publisher robot_state_pub = n.advertise<std_msgs::String>("/irb120/robot_state", 1000);        // Publisher to broadcast the current state of the robot
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
    MatrixXd H(4,4);

    while (ros::ok()){

        // Define vectors that represent the position of the PCB and error
        VectorXd desired_position(3);
        VectorXd ee_error(3);

        std_msgs::String state_msg;
        geometry_msgs::Point desired_position_msg;

        // Calculate the homogeneous matrix to get the current end-effector position
        H = move_bot.getHomogeneous(JointAngles, d, a, alpha);
        ee_current_position(0) = H(0, 3);
        ee_current_position(1) = H(1, 3);
        ee_current_position(2) = H(2, 3);

        switch(current_robot_state) {
            // =======================================================================================================================
            // This state initilizes the robot position above the PCB, a hardcoded position where the camera can easily detect the PCB
            // ======================================================================================================================= 
            case Initialization: 
                cout << "Initialization state ";
                // Set the position of the PCB
                desired_position(0) = -237;
                desired_position(1) = 384;
                desired_position(2) = 151;

                // use ee_position_pub to publish PCB position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg); 

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

    //            cout << "Error: [" << abs(ee_current_position(0)) << ", " << abs(ee_current_position(1)) << ", " << abs(ee_current_position(2)) << "]" << endl;

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = DetectPCB;
                }

                // Publish the robot current state
                state_msg.data = "Initialization";
                robot_state_pub.publish(state_msg);

                break;


                // =======================================================================================================================
                // ============== This state uses the computer vision node to obtain the position and orientation of the PCB =============
                // ======================================================================================================================= 
            case DetectPCB:

                // INSERT CODE FOR COMPUTER VISION
                // USE tf.can_transform to trigger the next state

                // Publish the robot current state
                state_msg.data = "DetectPCB";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // === This state moves the robot to a position above the SOIC, within the camera view to determine the SOIC position ====
                // ======================================================================================================================= 
            case Move2DetectSOIC:

                // Set the position of the SOIC
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to SOIC position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = DetectSOIC;
                }

                // Publish the robot current state
                state_msg.data = "Move2DetectSOIC";
                robot_state_pub.publish(state_msg);

                break;


                // =======================================================================================================================
                // ============= This state uses the computer vision node to obtain the position and orientation of the SOIC =============
                // ======================================================================================================================= 
            case DetectSOIC:

                // INSERT CODE FOR COMPUTER VISION
                // USE tf.can_transform to trigger the next state

                // Publish the robot current state
                state_msg.data = "DetectSOIC";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to the position of the syringe to pick it up =======================
                // ======================================================================================================================= 
            case Move2PickSyringe:

                // Set the position of the Syringe
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to SOIC position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = PickSyringe;
                }

                // Publish the robot current state
                state_msg.data = "Move2PickSyringe";
                robot_state_pub.publish(state_msg);

                break;

                // =======================================================================================================================
                // ======================== This state executes the pick up action to grip the syringe ===================================
                // ======================================================================================================================= 
            case PickSyringe:

                // INSERT CODE TO PICK UP SYRINGE
                // I'M NOT SURE HOW TO DO THIS YET

                // Publish the robot current state
                state_msg.data = "PickSyringe";
                robot_state_pub.publish(state_msg);

                break;

            case Move2ReleaseSolderPaste:
                // Set the position of the PCB to release solder paste
                // This position should be the one obtained by the CV node
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to PCB position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = ApplySolderPaste;
                }

                // Publish the robot current state
                state_msg.data = "Move2ReleaseSolderPaste";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ======================================= This state applies the solder paste ===========================================
                // =======================================================================================================================
            case ApplySolderPaste:
                // NOTE: WE WOULD HAVE A LIST OF NODES HERE TO APPLY THE SOLDER PASTE
                // Publish the robot current state
                state_msg.data = "ApplySolderPaste";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to the position of the syringe to drop it off ======================
                // ======================================================================================================================= 
            case Move2DropSyringe:
                // Set the position of the Syringe
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to syringe position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = DropSyringe;
                }

                // Publish the robot current state
                state_msg.data = "Move2DropSyringe";
                robot_state_pub.publish(state_msg);

                break;

                // =======================================================================================================================
                // ============================================ This state drops the syringe =========== =================================
                // =======================================================================================================================
            case DropSyringe:
                // Publish the robot current state
                state_msg.data = "DropSyringe";
                robot_state_pub.publish(state_msg);

                break;


                // =======================================================================================================================
                // ===================== This state moves the robot to the position of the suction cup to pick it up =====================
                // ======================================================================================================================= 
            case Move2PickSuction:

                // Set the position of the Suction cup to pick it up
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to suction cup position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = PickSuction;
                }

                // Publish the robot current state
                state_msg.data = "Move2PickSuction";
                robot_state_pub.publish(state_msg);


                break;


                // =======================================================================================================================
                // ========================================= This state picks up the suction cup =========================================
                // =======================================================================================================================
            case PickSuction:
                // Publish the robot current state
                state_msg.data = "PickSuction";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ======================== This state moves the robot to the position of the SOIC to pick it up =========================
                // ======================================================================================================================= 
            case Move2PickSOIC:
                // Set the position of the SOIC to pick it up
                // This position is the one obtained from the CV node
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to ee position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = PickSOIC;
                }

                // Publish the robot current state
                state_msg.data = "Move2PickSOIC";
                robot_state_pub.publish(state_msg);

                break;

                // =======================================================================================================================
                // ========================================== This state picks up the SOIC ===============================================
                // =======================================================================================================================
            case PickSOIC:
                // Publish the robot current state
                state_msg.data = "PickSOIC";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ====================== This state moves the robot to the position of the PCB to place the SOIC  =======================
                // ======================================================================================================================= 
            case Move2PlaceSOIC:
                // Set the position of the PCB to place the SOIC
                // This is the position obtained from the CV node
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to PCB position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = PlaceSOIC;
                }

                // Publish the robot current state
                state_msg.data = "Move2PlaceSOIC";
                robot_state_pub.publish(state_msg);

                break;

                // =======================================================================================================================
                // ====================================== This state places the SOIC on top of PCB =======================================
                // =======================================================================================================================
            case PlaceSOIC:
                // Publish the robot current state
                state_msg.data = "PlaceSOIC";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ====================== This state moves the robot to the position of the suction to drop it off  ======================
                // ======================================================================================================================= 
            case Move2DropSuction:
                // Set the position of the Suction cup to release
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to Suction cup position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = DropSuction;
                }

                // Publish the robot current state
                state_msg.data = "Move2DropSuction";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ========================================== This state drops the suction cup ===========================================
                // =======================================================================================================================
            case DropSuction:
                // Publish the robot current state
                state_msg.data = "DropSuction";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ==================== This state moves the robot to the position of the hot air pencil to pick it up ===================
                // ======================================================================================================================= 
            case Move2PickHotAirPencil:
                // Set the position of the Hot Air Pencil
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to Hot Air Pencil position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = PickHotAirPencil;
                }

                // Publish the robot current state
                state_msg.data = "Move2HotAirPencil";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ======================================= This state picks up the hot air pencil ========================================
                // =======================================================================================================================
            case PickHotAirPencil:
                // Publish the robot current state
                state_msg.data = "PickHotAirPencil";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ===================== This state moves the robot to the position of the PCB to perform soldering ======================
                // ======================================================================================================================= 
            case Move2SolderPCB:

                // Set the position of the PCB toapply hot air
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to PCB position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = ApplyHotAir;
                }

                // Publish the robot current state
                state_msg.data = "Move2SolderPCB";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ================================= This state applies hot air to melt the solder paste =================================
                // =======================================================================================================================
            case ApplyHotAir:
                // WE WOULD HAVE A LIST OF POINTS HERE CAUSE WE'RE MOVING THE HOT AIR AROUND
                // Publish the robot current state
                state_msg.data = "ApplyHotAir";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // =================== This state moves the robot to the position of the hot air pencil to drop it off ===================
                // ======================================================================================================================= 
            case Move2DropHotAirPencil:
                // Set the position of the Hot Air Pencil
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to Hot Air Pencil position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    current_robot_state = DropHotAirPencil;
                }

                // Publish the robot current state
                state_msg.data = "Move2DropHotAirPencil";
                robot_state_pub.publish(state_msg);
                break;

                // =======================================================================================================================
                // ========================================= This state releases the hot air pencil ======================================
                // =======================================================================================================================
            case DropHotAirPencil:
                // Publish the robot current state
                state_msg.data = "DropHotAirPencil";
                robot_state_pub.publish(state_msg);

                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to home position and terminates the program ========================
                // ======================================================================================================================= 
            case ReturnHome:
                // Set the home position
                desired_position(0) = 0.0;
                desired_position(1) = 0.0;
                desired_position(2) = 0.0;

                // Use ee_position_pub to move the robot to PCB position
                desired_position_msg.x = desired_position(0);
                desired_position_msg.y = desired_position(1);
                desired_position_msg.z = desired_position(2);
                ee_position_pub.publish(desired_position_msg);

                // Calculate the error between desired and real position
                // The real position is obtained by subscribing to the /joint_states topic published by the ABB controller
                ee_error = desired_position - ee_current_position; // Error = desired - real

                // Move to next state when the threshold is met
                if ((abs(ee_error(0)) < ERROR_THRESHOLD) && (abs(ee_error(1)) < ERROR_THRESHOLD) && (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    exit(EXIT_SUCCESS);
                }

                // Publish the robot current state
                state_msg.data = "ReturnHome";
                robot_state_pub.publish(state_msg);

                break;


        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << "Succesfully executed the tasks" << endl;

    return 0;
}

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

