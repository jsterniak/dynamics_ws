#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
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

geometry_msgs::Pose2D pcb_pose;
bool pcb_pose_found;

void PCBLocCB(const geometry_msgs::Pose2DConstPtr& msg)
{
    if(IRBStateMachine::DetectPCB == current_robot_state)
    {
      pcb_pose.x = msg->x;
      pcb_pose.y = msg->y;
      pcb_pose.theta = msg->theta;
      pcb_pose_found = true;
    }
    else
    {
      // do nothing
    }
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

    ros::Subscriber pcb_register_sub = n.subscribe("/irb120/pcb_pose", 100, PCBLocCB);

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

    pcb_pose.theta = 0.0;
    pcb_pose.x = 0.0;
    pcb_pose.y = 0.0;
    pcb_pose_found = false;

    std_msgs::Float64MultiArray ee_pose;
    ee_pose.data.resize(7);
    // Define vectors that represent the position of the PCB and error
    VectorXd desired_position(7);
    VectorXd real_position(3);
    VectorXd ee_error(3);
    VectorXd joint_error(6);
    while (ros::ok()){

        switch(current_robot_state) {
            // =======================================================================================================================
            // This state initilizes the robot position above the PCB, a hardcoded position where the camera can easily detect the PCB
            // ======================================================================================================================= 
            case IRBStateMachine::Initialization:
                {
                    cout << "Initialization State" << endl;

                    int n_via_point = 1;
                    double via_point[][6] =
                    {
                        {-M_PI, 0, -2.35874, -0.2585, 0.3431, 0.1489},
                      //  {-M_PI, 0, -2.35874, -0.2498, 0.3130, 0.1489}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        // Set the position of the PCB
                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //x //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //x; //z


                        H = move_bot.getHomogeneous(JointAngles, d, a, alpha);
                        //cout << H << endl;
                        //cout << "======" << endl;
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
                            via_p_cnt++;
                            cout << "threshold met" << endl;
                            current_robot_state = IRBStateMachine::DetectPCB;
                            //cout << "debug" << endl;
                        }

                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Initialization);
                        ros::spinOnce();
                    }


                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Initialization);
                exit(EXIT_SUCCESS);
                }
                break;


                // =======================================================================================================================
                // ============== This state uses the computer vision node to obtain the position and orientation of the PCB =============
                // ======================================================================================================================= 
            case IRBStateMachine::DetectPCB:

                cout << "Detecting PCB" << endl;

                if (pcb_pose_found)
                {
                  current_robot_state = IRBStateMachine::Move2DetectSOIC;
                }
                else
                {
                  // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::DetectPCB);

                break;


                // =======================================================================================================================
                // === This state moves the robot to a position above the SOIC, within the camera view to determine the SOIC position ====
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DetectSOIC:
                {
                    cout << "Move2DetectSOIC State" << endl;
                    int n_via_point = 2;
                    double via_point[][6] =
                    {
                        {-M_PI, 0, -2.35874, -0.1112, 0.2266, 0.188},
                        {-M_PI, 0, -2.35874, -0.1011, 0.2929, 0.1350}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2DetectSOIC State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::DetectSOIC;
                        }


                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2DetectSOIC);
                        ros::spinOnce();
                    }


                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2DetectSOIC);
                }
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
                    int n_via_point = 3;
                    double via_point[][6] = 
                    {
                        {-3.12769, 0.02070, 2.16441, 0.0041, 0.2257, 0.6224},
                        {-3.12769, 0.02070, 2.16441, 0.2255, 0.3362, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.2255, 0.3362, 0.3925}
                    };

                    //cout << "Move2PickSyringe State" << endl;
                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2PickSyringe State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                                (abs(ee_error(2)) < ERROR_THRESHOLD))  {
                            cout << "threshold met" << endl;
                            via_p_cnt++;
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
                {
                    // Set the position of the PCB to release solder paste
                    // This position should be the one obtained by the CV node
                    cout << "Move2ReleaseSolderPaste State" << endl;  

                    int n_via_point = 3;
                    double via_point[][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, 0.2255, 0.3362, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.0430, 0.2466, 0.4237},
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3106, 0.2843}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2ReleaseSolderPaste State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::ApplySolderPasteFirstSide;
                        }


                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2ReleaseSolderPaste);
                        ros::spinOnce();
                    }


                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2DetectSOIC);
                }
                break;

                // =======================================================================================================================
                // ======================================= This state applies the solder paste ===========================================
                // =======================================================================================================================
            case IRBStateMachine::ApplySolderPasteFirstSide:
                {
                    cout << "ApplySolderPasteFirstSide State" << endl;

                    int n_via_point = 7;

                    // Side 1
                    double via_point[][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3130, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3116, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3103, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3091, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3079, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3065, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.3051, 0.2843}

                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "ApplySolderPasteFirstSide State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++; 
                            current_robot_state = IRBStateMachine::ApplySolderPasteMoveSide;
                        }


                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::ApplySolderPasteFirstSide);
                        ros::spinOnce();
                    }

                    /*
                       if (last_actuated_state_ == current_robot_state)
                       {
                       current_robot_state = IRBStateMachine::Move2DropSyringe;
                       }
                       else
                       {
                    // do nothing
                    }
                     */
                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::ApplySolderPasteFirstSide);

                    //current_robot_state = IRBStateMachine::Move2DropSyringe; // for testing, delete after
                }
                break;

            case IRBStateMachine::ApplySolderPasteMoveSide:
                {
                    cout << "ApplySolderPasteMoveSide State" << endl;

                    int n_via_point = 2;


                    double via_point[][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, -0.1882, 0.2997, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3051, 0.2843}
                    //    {-3.12769, 0.02070, 2.16441, -0.1823, 0.2192, 0.2857}

                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "ApplySolderPasteMoveSide State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++; 
                            current_robot_state = IRBStateMachine::ApplySolderPasteSecondSide;
                        }


                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::ApplySolderPasteMoveSide);
                        ros::spinOnce();
                    }

                    /*
                       if (last_actuated_state_ == current_robot_state)
                       {
                       current_robot_state = IRBStateMachine::Move2DropSyringe;
                       }
                       else
                       {
                    // do nothing
                    }
                     */
                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::ApplySolderPasteMoveSide);

                    //current_robot_state = IRBStateMachine::Move2DropSyringe; // for testing, delete after
                }
                break;

            case IRBStateMachine::ApplySolderPasteSecondSide:
                {
                    cout << "ApplySolderPasteSecondSide State" << endl;

                    int n_via_point = 6;

                    // Side 2
                    double via_point[][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3064, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3075, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3090, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3101, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3114, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3127, 0.2843}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "ApplySolderPasteSecondSide State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++; 
                            current_robot_state = IRBStateMachine::Move2DropSyringe;
                        }


                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::ApplySolderPasteSecondSide);
                        ros::spinOnce();
                    }

                    /*
                       if (last_actuated_state_ == current_robot_state)
                       {
                       current_robot_state = IRBStateMachine::Move2DropSyringe;
                       }
                       else
                       {
                    // do nothing
                    }
                     */
                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::ApplySolderPasteSecondSide);
                    //exit(EXIT_SUCCESS);
                    //current_robot_state = IRBStateMachine::Move2DropSyringe; // for testing, delete after
                }
                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to the position of the syringe to drop it off ======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DropSyringe:
                {
                    // Set the position of the Syringe
                    cout << "Move2DropSyringe State" << endl;

                    int n_via_point = 5;
                    double via_point[][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3216, 0.2843},
                        {-3.12769, 0.02070, 2.16441, -0.1820, 0.3216, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.0041, 0.2257, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.2255, 0.3362, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.2255, 0.3362, 0.3925}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2DropSyringe State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z


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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::DropSyringe;
                        }


                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2DropSyringe);
                        ros::spinOnce();
                    }


                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2DropSyringe);
                }
                break;

                // =======================================================================================================================
                // ============================================ This state drops the syringe =========== =================================
                // =======================================================================================================================
            case IRBStateMachine::DropSyringe:
                cout << "DropSyringe" << endl;
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
                //current_robot_state = IRBStateMachine::Move2PickSuction; // for testing
                break;


                // =======================================================================================================================
                // ===================== This state moves the robot to the position of the suction cup to pick it up =====================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PickSuction:
                {
                    // Set the position of the Suction cup to pick it up
                    cout << "Move2PickSuction State" << endl;

                    int n_via_point = 3;
                    double via_point[3][6] =
                    {   
                        {-3.12769, 0.02070, 2.16441, 0.2255, 0.3362, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.1474, 0.3887, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.1474, 0.3887, 0.3925}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2PickSuction State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::PickSuction;
                        }
                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2PickSuction);
                        ros::spinOnce();
                    }


                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2PickSuction);
                }

                break;


                // =======================================================================================================================
                // ========================================= This state picks up the suction cup =========================================
                // =======================================================================================================================
            case IRBStateMachine::PickSuction:
                cout << "PickSuction State" << endl;
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
                current_robot_state = IRBStateMachine::Move2PickSOIC; // for testing
                break;

                // =======================================================================================================================
                // ======================== This state moves the robot to the position of the SOIC to pick it up =========================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PickSOIC:
                {                
                    // Set the position of the SOIC to pick it up
                    // This position is the one obtained from the CV node
                    cout << "Move2PickSOIC State" << endl;

                    int n_via_point = 3;
                    double via_point[][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, 0.1474, 0.3887, 0.4224},
                        {-3.14149, 0, -2.55214,-0.0426, 0.2894, 0.4224},
                        {-3.14149, 0, -2.55214, -0.0418, 0.2882, 0.1899}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2PickSOIC State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::PickSOIC;
                        }
                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2PickSOIC);
                        ros::spinOnce();
                    }

                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2PickSOIC);
                }
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
                current_robot_state = IRBStateMachine::Move2PlaceSOIC;
                break;

                // =======================================================================================================================
                // ====================== This state moves the robot to the position of the PCB to place the SOIC  =======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PlaceSOIC:
                {
                    // Set the position of the PCB to place the SOIC
                    // This is the position obtained from the CV node
                    cout << "Move2PlaceSOIC State" << endl;

                    int n_via_point = 3;
                    double via_point[][6] =
                    {
                        {-3.13636, 0, -3.08749, -0.0418, 0.2882, 0.2237},
                        {-3.13636, 0, -3.08749, -0.1890, 0.3115, 0.2265},
                        {-3.13636, 0, -3.08749, -0.1890, 0.3115, 0.2065}                    
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2PlaceSOIC State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::PlaceSOIC;
                        }
                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2PlaceSOIC);
                        ros::spinOnce();
                    }


                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2PlaceSOIC);
                }
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
                current_robot_state = IRBStateMachine::Move2DropSuction;
                break;

                // =======================================================================================================================
                // ====================== This state moves the robot to the position of the suction to drop it off  ======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DropSuction:
                { 
                    // Set the position of the Suction cup to release
                    cout << "Move2DropSuction State" << endl;

                    int n_via_point = 3;
                    double via_point[][6] =
                    {
                        {-3.12658, 0.02130, 2.16352, -0.0029, 0.3075, 0.5182},
                        {-3.12769, 0.02070, 2.16441, 0.1474, 0.3887, 0.4224},
                        {-3.12769, 0.02070, 2.16441, 0.1474, 0.3887, 0.3925}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2DropSuction State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::DropSuction;
                        }
                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2DropSuction);
                        ros::spinOnce();
                    }
                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2DropSuction);
                }                
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
                current_robot_state = IRBStateMachine::Move2PickHotAirPencil; //
                break;

                // =======================================================================================================================
                // ==================== This state moves the robot to the position of the hot air pencil to pick it up ===================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2PickHotAirPencil:
                {
                    // Set the position of the Hot Air Pencil
                    cout << "Move2PickHotAirPencil State" << endl;
                    int n_via_point = 3;
                    double via_point[3][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, 0.1474, 0.3887, 0.4224},
                        {-3.12769, 0.02070, 2.16441, -0.0025, 0.4900, 0.4203},
                        {-3.12769, 0.02070, 2.16441, -0.0025, 0.4900, 0.3925}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2PickHotAirPencil State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::PickHotAirPencil;
                        }
                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2PickHotAirPencil);
                        ros::spinOnce();
                    }
                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2PickHotAirPencil);
                }                
                break;

                // =======================================================================================================================
                // ======================================= This state picks up the hot air pencil ========================================
                // =======================================================================================================================
            case IRBStateMachine::PickHotAirPencil:

                cout << "PickHotAirPencil" << endl;
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
                current_robot_state = IRBStateMachine::Move2SolderPCB; //
                break;

                // =======================================================================================================================
                // ===================== This state moves the robot to the position of the PCB to perform soldering ======================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2SolderPCB:
                {
                    // Set the position of the PCB toapply hot air
                    cout << "Move2SolderPCB State" << endl;
                    int n_via_point = 3;
                    double via_point[3][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, -0.0025, 0.4900, 0.4203},
                        {-3.12769, 0.02070, 2.16441, -0.0844, 0.3590, 0.5018},
                        {-3.12769, 0.02070, 2.16441, -0.1890, 0.3120, 0.3939},
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2SolderPCB State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::ApplyHotAir;
                        }
                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2SolderPCB);
                        ros::spinOnce();
                    }

                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2SolderPCB);
                }
                break;

                // =======================================================================================================================
                // ================================= This state applies hot air to melt the solder paste =================================
                // =======================================================================================================================
            case IRBStateMachine::ApplyHotAir:
                if (last_actuated_state_ == current_robot_state)
                {
                    current_robot_state = IRBStateMachine::Move2DropHotAirPencil;
                }
                else
                {
                    // do nothing
                }

                // Publish the robot current state
                publish_state(robot_state_pub, IRBStateMachine::ApplyHotAir);
                current_robot_state = IRBStateMachine::Move2DropHotAirPencil;
                break;

                // =======================================================================================================================
                // =================== This state moves the robot to the position of the hot air pencil to drop it off ===================
                // ======================================================================================================================= 
            case IRBStateMachine::Move2DropHotAirPencil:
                {
                    // Set the position of the Hot Air Pencil
                    cout << "Move2DropHotAirPencil State" << endl;

                    int n_via_point = 3;
                    double via_point[][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, -0.0844, 0.3590, 0.5018}, 
                        {-3.12769, 0.02070, 2.16441, -0.0025, 0.4900, 0.4203},
                        {-3.12769, 0.02070, 2.16441, -0.0025, 0.4900, 0.3925}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "Move2DropHotAirPencil State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;
                            current_robot_state = IRBStateMachine::DropHotAirPencil;
                        }

                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::Move2DropHotAirPencil);
                        ros::spinOnce();
                    }


                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::Move2DropHotAirPencil);
                } 
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
                current_robot_state = IRBStateMachine::ReturnHome;
                break;

                // =======================================================================================================================
                // ======================= This state moves the robot to home position and terminates the program ========================
                // ======================================================================================================================= 
            case IRBStateMachine::ReturnHome:
                {
                    // Set the home position
                    cout << "ReturnHome State" << endl;

                    int n_via_point = 2;
                    double via_point[3][6] =
                    {
                        {-3.12769, 0.02070, 2.16441, -0.0025, 0.4900, 0.4203},
                        {-3.12769, 0.02070, 2.16441, 0.0041, 0.2570, 0.4224}
                    };

                    int via_p_cnt = 0;

                    while (via_p_cnt < n_via_point){
                        cout << "ReturnHome State " << via_p_cnt << endl;

                        tf::Quaternion desired_pose_raw;
                        desired_pose_raw = tf::createQuaternionFromRPY(via_point[via_p_cnt][0], via_point[via_p_cnt][1], via_point[via_p_cnt][2]);
                        tf::Quaternion robot_studio_to_ros;
                        robot_studio_to_ros = tf::createQuaternionFromRPY(0.0, -M_PI/2, 0.0);

                        tf::Quaternion desired_pose;
                        desired_pose = desired_pose_raw * robot_studio_to_ros;

                        desired_position(0) =  desired_pose.w(); //qw
                        desired_position(1) =  desired_pose.x(); //qx
                        desired_position(2) =  desired_pose.y(); //qy
                        desired_position(3) =  desired_pose.z(); //qz
                        desired_position(4) =  via_point[via_p_cnt][3]; //x
                        desired_position(5) =  via_point[via_p_cnt][4]; //y
                        desired_position(6) =  via_point[via_p_cnt][5]; //z

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
                            via_p_cnt++;

                            //current_robot_state = IRBStateMachine::DropHotAirPencil;
                        }

                        // Publish the robot current state
                        publish_state(robot_state_pub, IRBStateMachine::ReturnHome);
                        ros::spinOnce();
                    }

                    // Publish the robot current state
                    publish_state(robot_state_pub, IRBStateMachine::ReturnHome);
                    exit(EXIT_SUCCESS);
                }
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

