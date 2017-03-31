#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <eigen/Eigen/Dense>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

#define HOME_X 550
#define HOME_Y 200
#define HOME_Z 200

using namespace Eigen;
using namespace std;
class MoveRobot{
    private:
        // ============ D-H parameters ============
        VectorXd theta;
        VectorXd d;
        VectorXd a;
        VectorXd alpha;

        // Robot current EE position
        VectorXd current_pos;

        //std_msgs:: Float64 orient_msg;
        //geometry_msgs::Point tf;
        //geometry_msgs::Point position_msg;

        // ============ Joint Publishers ============
        ros::Publisher joint1_pub;
        ros::Publisher joint2_pub;
        ros::Publisher joint3_pub;
        ros::Publisher joint4_pub;
        ros::Publisher joint5_pub;
        ros::Publisher joint6_pub;

        // ============ Subscriber listening to new end effector position ============ 
        ros::Subscriber new_pos;


        //ros::Subscriber pos1_sub;
        //ros::Subscriber pos2_sub;
        //ros::Subscriber orientation_sub;
        //ros::Subscriber tf_sub;
        //ros::ServiceClient client1;
        //ros::ServiceClient client2;
        //double x_new;
        //double y_new;

    public:
        MoveRobot(VectorXd theta_, VectorXd d_, VectorXd a_, VectorXd alpha_, ros::NodeHandle n, VectorXd cur_pos)
        {
            theta = theta_;
            d = d_;
            a = a_;
            alpha = alpha_;

            theta(0) = theta(0)         * (M_PI/180);
            theta(1) = (theta(1) + 90)  * (M_PI/180);
            theta(2) = theta(2)         * (M_PI/180);
            theta(3) = theta(3)         * (M_PI/180);
            theta(4) = theta(4)         * (M_PI/180);
            theta(5) = theta(5)         * (M_PI/180);

            current_pos = cur_pos;

            joint1_pub = n.advertise<std_msgs::Float64>("/irb120/joint_1_position_controller/command",1000);
            joint2_pub = n.advertise<std_msgs::Float64>("/irb120/joint_2_position_controller/command",1000);
            joint3_pub = n.advertise<std_msgs::Float64>("/irb120/joint_3_position_controller/command",1000);
            joint4_pub = n.advertise<std_msgs::Float64>("/irb120/joint_4_position_controller/command",1000);
            joint5_pub = n.advertise<std_msgs::Float64>("/irb120/joint_5_position_controller/command",1000);
            joint6_pub = n.advertise<std_msgs::Float64>("/irb120/joint_6_position_controller/command",1000);

            new_pos = n.subscribe("/irb120/ee_pos", 100, &MoveRobot::NewPositionCallBack, this);
        }
        MoveRobot(VectorXd theta_, VectorXd d_, VectorXd a_, VectorXd alpha_)
        {
            theta = theta_;
            d = d_;
            a = a_;
            alpha = alpha_;

            theta(0) = theta(0)         * (M_PI/180);
            theta(1) = (theta(1) + 90)  * (M_PI/180);
            theta(2) = theta(2)         * (M_PI/180);
            theta(3) = theta(3)         * (M_PI/180);
            theta(4) = theta(4)         * (M_PI/180);
            theta(5) = theta(5)         * (M_PI/180);

        }
        MatrixXd MatrixTransformation(double theta, double d, double a, double alpha);
        MatrixXd getR03(VectorXd theta_);
        MatrixXd getHomogeneous(MatrixXd theta, VectorXd d, VectorXd a, VectorXd alpha);
        MatrixXd InverseKinematics(MatrixXd H);
        void moveRobot(MatrixXd theta);
        MatrixXd Trajectory(MatrixXd i_pos, MatrixXd f_pos, double theta_deg);
        MatrixXd QuadGen(double theta_i, double theta_f, double ang_v, double t_start, double t_final);
        MatrixXd CubGen(double theta_i, double theta_f, double ang_vi, double ang_vf, double t_start, double t_final);
        MatrixXd Polynome(MatrixXd q, MatrixXd t, double v_i, double a_i, double v_f, double a_f);
        void NewPositionCallBack (const geometry_msgs::Point pos);

};
