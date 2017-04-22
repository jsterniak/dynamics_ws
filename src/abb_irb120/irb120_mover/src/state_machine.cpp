#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include "MoveRobot.h"
#include "state_machine.hpp"

#define ERROR_THRESHOLD 0.1

IRBStateMachine::RobotState current_robot_state;

MatrixXd JointAngles(1,6);          // The robot current joint angles
VectorXd ee_current_position(3);    // The robot real position of the end-effector calculated from the JointAngles

void EEPositionFeedBackCB (const sensor_msgs::JointState msg)
{   
/*    
       cout << "Real Angle Feedback: ";
       cout << msg.position[0] << ", ";
       cout << msg.position[1] << ", ";
       cout << msg.position[2] << ", ";
       cout << msg.position[3] << ", ";
       cout << msg.position[4] << ", ";
       cout << msg.position[5] << endl;
  */   
    JointAngles(0,0)=   msg.position[0];
    JointAngles(0,1)=   msg.position[1];
    JointAngles(0,2)=   msg.position[2];
    JointAngles(0,3)=   msg.position[3];
    JointAngles(0,4)=   msg.position[4];
    JointAngles(0,5)=   msg.position[5];
}

IRBStateMachine::RobotState last_actuated_state_;

void ActuationStateCB (const std_msgs::Int32ConstPtr& msg)
{
    last_actuated_state_ = static_cast<IRBStateMachine::RobotState>(msg->data);
}

void publish_state(const ros::Publisher& f_pub, const IRBStateMachine::RobotState f_state)
{
    std_msgs::Int32 state_msg;
    state_msg.data = f_state;
    f_pub.publish(state_msg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "irb120_state_machine");
    ros::NodeHandle n;
    ros::Publisher robot_state_pub = n.advertise<std_msgs::Int32>("/irb120/robot_state", 1000);        // Publisher to broadcast the current state of the robot
    ros::Publisher ee_position_pub = n.advertise<std_msgs::Float64MultiArray>("/irb120/ee_pose", 1000); // Publisher to broadcast the position the EE needs to be at

    ros::Subscriber ee_pos_sub  = n.subscribe("/joint_states", 100, EEPositionFeedBackCB);      // Subscriber to get the real robot JointAngles
    ros::Subscriber actuator_state_sub = n.subscribe("/irb120/actuation_state_complete", 100, ActuationStateCB);

    ros::Rate loop_rate(20); // Publishing at 50 ms = 20Hz

    // Initializing D-H parameters of the robot
    VectorXd theta(6);
    VectorXd d(6);
    VectorXd a(6);
    VectorXd alpha(6);
    VectorXd cur_pos(3);

    d(0) = 290; 
    d(1) = 0; 
    d(2) = 0; 
    d(3) = 302; 
    d(4) = 0; 
    d(5) = 72;

    a(0) = 0; 
    a(1) = 270; 
    a(2) = 70; 
    a(3) = 0; 
    a(4) = 0; 
    a(5) = 0;

    alpha(0) = -90 * (M_PI/180.0); 
    alpha(1) = 0 * (M_PI/180.0); 
    alpha(2) = -90 * (M_PI/180.0);
    alpha(3) = 90 * (M_PI/180.0); 
    alpha(4) = -90 * (M_PI/180.0); 
    alpha(5) = 0 * (M_PI/180.0);

    // MoveBot object to calculate Kinematics
    MoveRobot move_bot(theta, d, a, alpha);
    move_bot.setCurJA(M_PI/2, 0, 0, 0, -M_PI/2, 0);
    move_bot.setCurPos(0, 302, 558);
    // The homogeneous transformation matrix H from base to tip
    MatrixXd H(4,4);

    // initialize states
    current_robot_state = IRBStateMachine::Initialization;
    last_actuated_state_ = IRBStateMachine::Initialization;

    std_msgs::Float64MultiArray ee_pose;
    ee_pose.data.resize(7);
    // Define vectors that represent the position of the PCB and error
    VectorXd desired_position(7);
    VectorXd real_position(3);
    VectorXd ee_error(3);
    while (ros::ok()){

        switch(current_robot_state) {
            // =======================================================================================================================
            // This state initilizes the robot position above the PCB, a hardcoded position where the camera can easily detect the PCB
            // ======================================================================================================================= 
            case IRBStateMachine::Initialization:
                cout << "Initialization State" << endl;
                // Set the position of the PCB
                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.304; //x
                desired_position(5) =      0.346; //y
                desired_position(6) =      0.160; //z

                H = move_bot.getHomogeneous(JointAngles, d, a, alpha);

                ee_pose.data[0] = desired_position(0);
                ee_pose.data[1] = desired_position(1);
                ee_pose.data[2] = desired_position(2);
                ee_pose.data[3] = desired_position(3);
                ee_pose.data[4] = desired_position(4);
                ee_pose.data[5] = desired_position(5);
                ee_pose.data[6] = desired_position(6);

                ee_position_pub.publish(ee_pose);

                real_position(0) = H(0, 3);
                real_position(1) = H(1, 3);
                real_position(2) = H(2, 3);

                ee_error(0) = real_position(0) - 1000 * desired_position(4);
                ee_error(1) = real_position(1) - 1000 * desired_position(5);
                ee_error(2) = real_position(2) - 1000 * desired_position(6);

                if ((abs(ee_error(0)) < ERROR_THRESHOLD) &&
                    (abs(ee_error(1)) < ERROR_THRESHOLD) &&
                    (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::DetectPCB;
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Initialization);

                break;


                // =======================================================================================================================
                // ============== This state uses the computer vision node to obtain the position and orientation of the PCB =============
                // ======================================================================================================================= 
            case IRBStateMachine::DetectPCB:

                // INSERT CODE FOR COMPUTER VISION
                // USE tf.can_transform to trigger the next state
                cout << "Detecting PCB" << endl;

                for (int i = 0; i < 20; i++){
                    cout << "Detecting PCB" << endl;
                }

                current_robot_state = IRBStateMachine::Move2DetectSOIC;

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::DetectPCB);
                ;
                break;

                // =======================================================================================================================
                // === This state moves the robot to a position above the SOIC, within the camera view to determine the SOIC position ====
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DetectSOIC:

                cout << "Move2DetectSOIC State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =      0.0375; //x
                desired_position(5) =      0.337; //y
                desired_position(6) =      0.144; //z

                H = move_bot.getHomogeneous(JointAngles, d, a, alpha);

                ee_pose.data[0] = desired_position(0);
                ee_pose.data[1] = desired_position(1);
                ee_pose.data[2] = desired_position(2);
                ee_pose.data[3] = desired_position(3);
                ee_pose.data[4] = desired_position(4);
                ee_pose.data[5] = desired_position(5);
                ee_pose.data[6] = desired_position(6);

                ee_position_pub.publish(ee_pose);

                real_position(0) = H(0, 3);
                real_position(1) = H(1, 3);
                real_position(2) = H(2, 3);

                ee_error(0) = real_position(0) - 1000 * desired_position(4);
                ee_error(1) = real_position(1) - 1000 * desired_position(5);
                ee_error(2) = real_position(2) - 1000 * desired_position(6);

                if ((abs(ee_error(0)) < ERROR_THRESHOLD) &&
                    (abs(ee_error(1)) < ERROR_THRESHOLD) &&
                    (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::DetectSOIC;
                }
                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2DetectSOIC);

                break;


                // =======================================================================================================================
                // ============= This state uses the computer vision node to obtain the position and orientation of the SOIC =============
                // ======================================================================================================================= 
            case IRBStateMachine::DetectSOIC:

                // INSERT CODE FOR COMPUTER VISION
                // USE tf.can_transform to trigger the next state
                for (int i = 0; i < 20; i++){
                    cout << "Detecting SOIC" << endl;
                }

                current_robot_state = IRBStateMachine::Move2PickSyringe;
                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::DetectSOIC);
                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to the position of the syringe to pick it up =======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PickSyringe:
                {
                    //point 1: 0.0375, 0.337, 0.502
                    //point 2: -0.023, 0.437, 0.502
                    //point 3: -0.023, 0.437, 0.3885
                    double via_point[3][7] = {{sqrt(2)/2, 0, sqrt(2)/2, 0, 0.0375, 0.337, 0.502},
                        {sqrt(2)/2, 0, sqrt(2)/2, 0, -0.023, 0.437, 0.502},
                        {sqrt(2)/2, 0, sqrt(2)/2, 0, -0.023, 0.437, 0.3885}};

                    //cout << "Move2PickSyringe State" << endl;
                    int via_p = 0;

                    while (via_p < 3){
                        cout << "Move2PickSyringe State " << via_p << endl;

                        desired_position(0) =  via_point[via_p][0]; //qw
                        desired_position(1) =  via_point[via_p][1]; //qx
                        desired_position(2) =  via_point[via_p][2]; //qy
                        desired_position(3) =  via_point[via_p][3]; //qz
                        desired_position(4) =  via_point[via_p][4]; //x
                        desired_position(5) =  via_point[via_p][5]; //y
                        desired_position(6) =  via_point[via_p][6]; //z

                        H = move_bot.getHomogeneous(JointAngles, d, a, alpha);

                        ee_pose.data[0] = desired_position(0);
                        ee_pose.data[1] = desired_position(1);
                        ee_pose.data[2] = desired_position(2);
                        ee_pose.data[3] = desired_position(3);
                        ee_pose.data[4] = desired_position(4);
                        ee_pose.data[5] = desired_position(5);
                        ee_pose.data[6] = desired_position(6);

                        ee_position_pub.publish(ee_pose);

                        real_position(0) = H(0, 3);
                        real_position(1) = H(1, 3);
                        real_position(2) = H(2, 3);

                        ee_error(0) = real_position(0) - 1000 * desired_position(4);
                        ee_error(1) = real_position(1) - 1000 * desired_position(5);
                        ee_error(2) = real_position(2) - 1000 * desired_position(6);
                        
                        if ((abs(ee_error(0)) < ERROR_THRESHOLD) &&
                            (abs(ee_error(1)) < ERROR_THRESHOLD) &&
                            (abs(ee_error(2)) < ERROR_THRESHOLD)) {
                            cout << "threshold met" << endl;
                            via_p++;
                        }


                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2PickSyringe);
                        ros::spinOnce();
                    }

                    current_robot_state = IRBStateMachine::PickSyringe;
                }
                break;

                // =======================================================================================================================
                // ======================== This state executes the pick up action to grip the syringe ===================================
                // ======================================================================================================================= 
            case IRBStateMachine::PickSyringe:
                
                cout << "PickSyringe State " << endl;
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2ReleaseSolderPaste;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::PickSyringe);

                break;

            case IRBStateMachine::Move2ReleaseSolderPaste:
                // Set the position of the PCB to release solder paste
                // This position should be the one obtained by the CV node
                cout << "Move2ReleaseSolderPaste State" << endl;  

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::ApplySolderPaste;
                }


                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2ReleaseSolderPaste);
                break;

                // =======================================================================================================================
                // ======================================= This state applies the solder paste ===========================================
                // =======================================================================================================================
            case IRBStateMachine::ApplySolderPaste:

                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2DropSyringe;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::ApplySolderPaste);
                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to the position of the syringe to drop it off ======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DropSyringe:
                // Set the position of the Syringe
                cout << "Move2DropSyringe State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::DropSyringe;
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2DropSyringe);

                break;

                // =======================================================================================================================
                // ============================================ This state drops the syringe =========== =================================
                // =======================================================================================================================
            case IRBStateMachine::DropSyringe:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2PickSuction;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::DropSyringe);

                break;


                // =======================================================================================================================
                // ===================== This state moves the robot to the position of the suction cup to pick it up =====================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PickSuction:

                // Set the position of the Suction cup to pick it up
                cout << "Move2PickSuction State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::PickSuction;
                }
                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2PickSuction);


                break;


                // =======================================================================================================================
                // ========================================= This state picks up the suction cup =========================================
                // =======================================================================================================================
            case IRBStateMachine::PickSuction:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2PickSOIC;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::PickSuction);
                break;

                // =======================================================================================================================
                // ======================== This state moves the robot to the position of the SOIC to pick it up =========================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PickSOIC:
                // Set the position of the SOIC to pick it up
                // This position is the one obtained from the CV node
                cout << "Move2PickSOIC State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::PickSOIC;
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2PickSOIC);

                break;

                // =======================================================================================================================
                // ========================================== This state picks up the SOIC ===============================================
                // =======================================================================================================================
            case IRBStateMachine::PickSOIC:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2PlaceSOIC;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::PickSOIC);
                break;

                // =======================================================================================================================
                // ====================== This state moves the robot to the position of the PCB to place the SOIC  =======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PlaceSOIC:
                // Set the position of the PCB to place the SOIC
                // This is the position obtained from the CV node
                cout << "Move2PlaceSOIC State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::PlaceSOIC;
                }                

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2PlaceSOIC);

                break;

                // =======================================================================================================================
                // ====================================== This state places the SOIC on top of PCB =======================================
                // =======================================================================================================================
            case IRBStateMachine::PlaceSOIC:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2DropSuction;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::PlaceSOIC);
                break;

                // =======================================================================================================================
                // ====================== This state moves the robot to the position of the suction to drop it off  ======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DropSuction:
                // Set the position of the Suction cup to release
                cout << "Move2DropSuction State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::DropSuction;
                }


                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2DropSuction);
                break;

                // =======================================================================================================================
                // ========================================== This state drops the suction cup ===========================================
                // =======================================================================================================================
            case IRBStateMachine::DropSuction:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2PickHotAirPencil;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::DropSuction);
                break;

                // =======================================================================================================================
                // ==================== This state moves the robot to the position of the hot air pencil to pick it up ===================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PickHotAirPencil:
                // Set the position of the Hot Air Pencil
                cout << "Move2PickHotAirPencil State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::PickHotAirPencil;
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2PickHotAirPencil);
                break;

                // =======================================================================================================================
                // ======================================= This state picks up the hot air pencil ========================================
                // =======================================================================================================================
            case IRBStateMachine::PickHotAirPencil:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2SolderPCB;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::PickHotAirPencil);
                break;

                // =======================================================================================================================
                // ===================== This state moves the robot to the position of the PCB to perform soldering ======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2SolderPCB:

                // Set the position of the PCB toapply hot air
                cout << "Move2SolderPCB State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::ApplyHotAir;
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2SolderPCB);
                break;

                // =======================================================================================================================
                // ================================= This state applies hot air to melt the solder paste =================================
                // =======================================================================================================================
            case IRBStateMachine::ApplyHotAir:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2ReleaseSolderPaste;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::ApplyHotAir);
                break;

                // =======================================================================================================================
                // =================== This state moves the robot to the position of the hot air pencil to drop it off ===================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DropHotAirPencil:
                // Set the position of the Hot Air Pencil
                cout << "Move2DropHotAirPencil State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "threshold met" << endl;
                    current_robot_state = IRBStateMachine::DropHotAirPencil;
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::Move2DropHotAirPencil);
                break;

                // =======================================================================================================================
                // ========================================= This state releases the hot air pencil ======================================
                // =======================================================================================================================
            case IRBStateMachine::DropHotAirPencil:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::ReturnHome;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::DropHotAirPencil);

                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to home position and terminates the program ========================
                // ======================================================================================================================= 
            case IRBStateMachine::ReturnHome:
                // Set the home position
                cout << "ReturnHome State" << endl;

                desired_position(0) =  sqrt(2)/2; //qw
                desired_position(1) =          0; //qx
                desired_position(2) =  sqrt(2)/2; //qy
                desired_position(3) =          0; //qz
                desired_position(4) =     -0.174; //x
                desired_position(5) =      0.415; //y
                desired_position(6) =      0.145; //z

                if ((abs(ee_error(0) < 5)) &&
                        (abs(ee_error(1) < 5)) &&
                        (abs(ee_error(2) < 5))) {
                    cout << "Succesfully Executed" << endl;
                    exit(EXIT_SUCCESS);
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::ReturnHome);

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

