#include "MoveRobot.h"


#define HOME_X 350
#define HOME_Y -200
#define HOME_Z 400

//Function for generating Transformation Matrix
MatrixXd MoveRobot::MatrixTransformation(double theta, double d_, double a_, double alpha_)
{
    MatrixXd T01(4,4);
    T01(0,0) = cos(theta);  T01(0,1) = -sin(theta)*cos(alpha_); T01(0,2) = sin(theta)*sin(alpha_);  T01(0,3) = a_ * cos(theta);
    T01(1,0) = sin(theta);  T01(1,1) = cos(theta)*cos(alpha_);  T01(1,2) = -cos(theta)*sin(alpha_); T01(1,3) = a_ * sin(theta);
    T01(2,0) = 0;           T01(2,1) = sin(alpha_);             T01(2,2) = cos(alpha_);             T01(2,3) = d_;
    T01(3,0) = 0;           T01(3,1) = 0;                       T01(3,2) = 0;                       T01(3,3) = 1;
    return T01;
}
//Inverse Kinematics Function - Given any H(4x4) Matrix to get all the 6 Joint values
MatrixXd MoveRobot::InverseKinematics(MatrixXd H)
{
    int i, j, index;
    MatrixXd Pos(3,1);
    MatrixXd Rot(3,3);
    MatrixXd k(3,1);
    MatrixXd Pos1(3,1);
    MatrixXd JointAngles(1,6);
    MatrixXd R03(3,3);
    MatrixXd R36(3,3);
    MatrixXd E_Pos(3,1);
    double x, y, z, R, alpha, beta, theta1,theta2, theta3, theta4, theta5, theta6, C2, S2, temp;
    for (i=0;i<=2;i++)
    {
        Pos(i,0)=H(i,3);
        if (i==2)
        {
            k(i,0)=1;
        }
        else
        {
            k(i,0)=0;
        }
        for (j=0;j<=2;j++)
        {
            Rot(i,j)=H(i,j);
        }
    }

    MatrixXd b(3,1);
    b(0,0) = 0;
    b(1,0) = 1;
    b(2,0) = 1;
    MatrixXd end(3,1);
    // E_Pos = Rot*b;
    Pos(0,0) = Pos(0,0);
    Pos(1,0) = Pos(1,0)-(32.5);
    Pos(2,0) = Pos(2,0)+(165);

    Pos1 = Pos - (72) * Rot * k;
    x = Pos1(0,0);
    y = Pos1(1,0);
    z = Pos1(2,0);

    theta1=atan2(y,x);
    R=sqrt(x*x+y*y);
    alpha=atan2(70,302);
    beta=atan2(R,z-290);
    C2=(pow(R,2)+ pow(z-290,2)+pow(270,2)-pow(302,2)-pow(70,2))/(540*sqrt(pow(R,2)+pow(z-290,2)));
    S2=sqrt(1-pow(C2,2));

    theta2=atan2(S2,C2)-beta;

    temp=atan2(z-290-270*cos(theta2), R+270*sin(theta2));
    theta3=temp-theta2-alpha;

    VectorXd t(3);
    t(0) = theta1; t(1) = theta2+(90*M_PI/180.0); t(2) = theta3;
    R03 = getR03(t);
    R36 = R03.transpose() * Rot;
    double S5 = sqrt(pow(R36(0,2),2) + pow(R36(1,2),2));
    theta5 = atan2(S5 , R36(2,2));

    theta4 = atan2(R36(1,2)/S5,R36(0,2)/S5);

    theta6 = atan2(R36(2,1)/S5, -R36(2,0)/S5);

    JointAngles(0,0)=theta1;
    JointAngles(0,1)=theta2;
    JointAngles(0,2)=theta3;
    JointAngles(0,3)=theta4;
    JointAngles(0,4)=theta5;
    JointAngles(0,5)=theta6;

    return JointAngles;
}
//Function for generation the Final Homogeneous Matrix
MatrixXd MoveRobot::getHomogeneous(MatrixXd q_values, VectorXd d, VectorXd a, VectorXd alpha)
{
    Matrix4d T06;
    MatrixXd T01;
    MatrixXd T12;
    MatrixXd T23;
    MatrixXd T34;
    MatrixXd T45;
    MatrixXd T56;

    MatrixXd theta(1,6);

    theta(0,0) = q_values(0,0);
    theta(0,1) = q_values(0,1) - (90 * (M_PI/180));
    theta(0,2) = q_values(0,2);
    theta(0,3) = q_values(0,3);
    theta(0,4) = q_values(0,4);
    theta(0,5) = q_values(0,5) + (M_PI);


    T01 = MatrixTransformation(theta(0,0),d(0),a(0),alpha(0));
    T12 = MatrixTransformation(theta(0,1),d(1),a(1),alpha(1));
    T23 = MatrixTransformation(theta(0,2),d(2),a(2),alpha(2));
    T34 = MatrixTransformation(theta(0,3),d(3),a(3),alpha(3));
    T45 = MatrixTransformation(theta(0,4),d(4),a(4),alpha(4));
    T56 = MatrixTransformation(theta(0,5),d(5),a(5),alpha(5));
    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    return T06;
}

//Gives R03 matrix required for Inverse Kinematics calculation
MatrixXd MoveRobot::getR03(VectorXd theta_)
{
    MatrixXd R03(3,3);
    MatrixXd T03;
    MatrixXd T01;
    MatrixXd T12;
    MatrixXd T23;

    T01 = MatrixTransformation(theta_(0),d(0),a(0),alpha(0));
    T12 = MatrixTransformation(theta_(1),d(1),a(1),alpha(1));
    T23 = MatrixTransformation(theta_(2),d(2),a(2),alpha(2));
    T03 = T01 * T12 * T23;
    for (int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R03(i,j) = T03(i,j);
        }
    }
    return R03;
}

//This function publishes the recieved joint angles to the robot correspondingly. 
void MoveRobot::moveRobot(MatrixXd theta)
{
    std_msgs::Float64 msg1,msg2,msg3,msg4,msg5,msg6;
    std_msgs::Float64MultiArray joint_angles;
    joint_angles.data.resize(6);

    // cout << "Moving Robot" << endl;


    if (!(isnan(theta(0,0)))) {cur_joint_values(0) = theta(0,0);}
    if (!(isnan(theta(0,1)))) {cur_joint_values(1) = theta(0,1);}
    if (!(isnan(theta(0,2)))) {cur_joint_values(2) = theta(0,2);}
    if (!(isnan(theta(0,3)))) {cur_joint_values(3) = theta(0,3);}
    if (!(isnan(theta(0,4)))) {cur_joint_values(4) = theta(0,4);}
    if (!(isnan(theta(0,5)))) {cur_joint_values(5) = theta(0,5);}


    joint_angles.data[0] = theta(0,0);//cur_joint_values(0);
    joint_angles.data[1] = theta(0,1);//cur_joint_values(1);
    joint_angles.data[2] = theta(0,2);//cur_joint_values(2);
    joint_angles.data[3] = theta(0,3);//cur_joint_values(3);
    joint_angles.data[4] = theta(0,4);//cur_joint_values(4);
    joint_angles.data[5] = theta(0,5);//cur_joint_values(5);
/*
    cout << "Move Robot publishing: ";
    cout << "[" << cur_joint_values(0);
    cout << ", "<< cur_joint_values(1);
    cout << ", "<< cur_joint_values(2);
    cout << ", "<< cur_joint_values(3);
    cout << ", "<< cur_joint_values(4);
    cout << ", "<< cur_joint_values(5) << "]" << endl; 
*/
    all_joints_pub.publish(joint_angles);


}

//This functions does the Trajectory planning that makes the robot move to the required point.
MatrixXd MoveRobot::Trajectory(MatrixXd i_pos, MatrixXd f_pos, double theta_deg)
{

    //cout << "Received " + i_pos << endl;

    double theta_rad = theta_deg * M_PI/180;
    double n_via = 10;
    double t = 10;
    double f_time = 0.5;
    double dist;
    MatrixXd diff(3,1);
    MatrixXd dir(3,1);
    diff = (f_pos - i_pos);
    dist = sqrt(pow(diff(0,0),2) + pow(diff(1,0),2) + pow(diff(2,0),2));
    dir(0,0) = (diff(0,0))/dist;
    dir(1,0) = (diff(1,0))/dist;
    dir(2,0) = (diff(2,0))/dist;
    double v_max = dist/(t*(1-f_time));
    MatrixXd orientation(3,3);

    orientation(0,0)= cos(theta_rad);
    orientation(0,1)= -sin(theta_rad);
    orientation(0,2)= 0;
    orientation(1,0)= -sin(theta_rad);
    orientation(1,1)= -cos(theta_rad);
    orientation(1,2)= 0;
    orientation(2,0)= 0;
    orientation(2,1)= 0;
    orientation(2,2)= -1;

    MatrixXd vel(15,1);
    MatrixXd S(15,1);
    MatrixXd position(15,1);
    MatrixXd H(4,4);
    MatrixXd JointAngles(1,6);
    MatrixXd Joint_Angles(11,6);
    MatrixXd time(11,1);

    for (int i=0;i<=n_via;i++)
    {
        if (i==0)
        {
            vel(i,0)=0;
            S(i,0)=0;
        }
        else if (i==n_via)
        {
            vel(i,0)=0;
            S(i,0)=dist;
        }
        else if (i<=(f_time*n_via))
        {
            vel(i,0)=v_max*((i)/(f_time*n_via));
            S(i,0)=0.5*vel(i,0)*(i)*t/n_via;
        }
        else if (i>=(1-f_time)*n_via)
        {
            vel(i,0)=v_max*(n_via-i)/(n_via*f_time);
            S(i,0)=0.5*v_max*((2*(1-f_time)*t)-(f_time*t))+0.5*(t/n_via)*(i-(1-f_time)*n_via)*(v_max+vel(i,0));
        }
        else
        {
            vel(i,0)=v_max;
            S(i,0)=0.5*v_max*((2*(i)*t/n_via)-(f_time*t));
        }

        position = i_pos + (dir * S(i))  ;

        for (int j=0;j<4;j++)
        {
            for (int k=0;k<4;k++)
            {
                if(j<3 && k <3)
                {
                    H(j,k)=orientation(j,k);
                }
                if(j<3 && k == 3)
                {
                    H(j,k)=position(j,0);
                }
                if(k<3 && j == 3)
                {
                    H(j,k) = 0;
                }
                if(k == 3 && j == 3)
                {
                    H(j,k) = 1;
                }
            }
            time(i,0)=i*t/n_via;
        }

        JointAngles = InverseKinematics(H);
        for(int j = 0;j<6;j++)
        {
            Joint_Angles(i,j) = JointAngles(0,j);
        }
        if (std::abs(Joint_Angles(i,4))<=0.0349066)
        {
            double NewJointAngle6=Joint_Angles(i-1,5)+(Joint_Angles(i,3)+Joint_Angles(i,5)-Joint_Angles(i-1,3)-Joint_Angles(i-1,5));
            Joint_Angles(i,3)=Joint_Angles(i-1,3);
            Joint_Angles(i,5)=NewJointAngle6;
        }
        for(int k=0;k<6;k++)
        {
            if(i>=1)
            {
                if((abs(Joint_Angles(i,k) - Joint_Angles(i-1,k))) > 4.71239)
                {
                    if(Joint_Angles(i,k)<0)
                    {
                        Joint_Angles(i,k) = Joint_Angles(i,k) + 6.28319;
                    }
                    else if(Joint_Angles(i,k)>=0)
                    {
                        Joint_Angles(i,k) = Joint_Angles(i,k) - 6.28319;
                    }
                }
            }
        }
    }


    MatrixXd Coeff(15,1);
    MatrixXd timeMatrix(1,15);
    MatrixXd q(11,1);
    MatrixXd Angles2(1001,6);
    MatrixXd Try(1,1);
    int index;
    for (int j=0; j<=5; j++)
    {
        index=0;
        for (int i=0; i<=10; i++)
        {
            q(i,0)=Joint_Angles(i,j);
        }
        Coeff=Polynome(q, time, 0.0, 0.0, 0.0, 0.0);

        for (double time1=0; time1<=10; time1=time1+0.01)
        {
            for (int k=0; k<=14; k++)
            {
                if (time1==0)
                {
                    timeMatrix(0,k)=0;
                }
                else
                {
                    timeMatrix(0,k)=pow(time1,k);
                }
                timeMatrix(0,0)=1;
            }
            Try=timeMatrix * Coeff;
            Angles2(index,j)=Try(0,0);
            index=index+1;
        }
    }

    MatrixXd angles(1,6);
    ros::Rate loop_rate(500);
    for(int i = 0; i<1000;i++)
    {
        for(int j=0;j<6;j++)
        {
            angles(0,j) = Angles2(i,j);
        }
        moveRobot(angles);
        loop_rate.sleep();
    }

    return Angles2;
}

//This function generates co-efficient matrix for Trajectory Planning. Trajectory is planned using a 14-degree polynomial with via-points added as constraints
MatrixXd MoveRobot::Polynome(MatrixXd q, MatrixXd t, double v_i, double a_i, double v_f, double a_f)
{
    MatrixXd A(15,15);
    MatrixXd B(15,1);
    MatrixXd X(15,1);
    for (int i=0; i<=14; i++)
    {
        for (int j=0; j<=14;j++)
        {
            A(i,j)=0;
        }
    }
    for (int i=0; i<=10; i++)
    {
        for (int j=0; j<=14; j++)
        {
            A(i,j)=pow(t(i,0), j);

        }
    }

    A(11,1)=1;
    A(12,2)=2;
    for (int j=0; j<=14; j++)
    {
        A(13,j)=(j)*pow(t(10,0),(j-1));
        A(14,j)=(j)*(j-1)*pow(t(10,0),(j-2));
    }
    for (int i=0; i<=10; i++)
    {
        B(i,0)=q(i);
    }
    B(11,0)=v_i;
    B(12,0)=a_i;
    B(13,0)=v_f;
    B(14,0)=a_f;

    X=A.inverse() * B;
    return X;
}

void MoveRobot::NewPositionCallBack(const std_msgs::Float64MultiArray msg)
{
    //ROS_INFO("Got new pose");

    /*
    MatrixXd new_position(3, 1);
    MatrixXd cur_pos (3, 1);

    new_position(0, 0) = pos_msg.x;
    new_position(1, 0) = pos_msg.y;
    new_position(2, 0) = pos_msg.z;

    cur_pos(0, 0) = current_pos(0);
    cur_pos(1, 0) = current_pos(1);
    cur_pos(2, 0) = current_pos(2);
    
       cout << "Current [";
       cout << current_pos(0);
       cout << ", ";
       cout << current_pos(1);
       cout << ", ";
       cout << current_pos(2) << "]";

       cout << " New [";
       cout << new_position(0);
       cout << ", ";
       cout << new_position(1);
       cout << ", ";
       cout << new_position(2) << "]" << endl;
     */


    /*
       if (!( (e(0, 0) == 0) && (e(1, 0) == 0) && (e(2,0) == 0) ))
       {
       Trajectory(cur_pos, new_position, 0.0);
       }
     */
   
    //Trajectory(cur_pos, new_position, 0.0);

    MatrixXd ja(1,6);
    ja(0,0) = msg.data[0];
    ja(0,1) = msg.data[1];
    ja(0,2) = msg.data[2];
    ja(0,3) = msg.data[3];
    ja(0,4) = msg.data[4];
    ja(0,5) = msg.data[5];
    moveRobot(ja);
    cout << ja << endl;
}

void MoveRobot::setCurJA(double j0, double j1, double j2, double j3, double j4, double j5){
    cur_joint_values(0) = j0;
    cur_joint_values(1) = j1;
    cur_joint_values(2) = j2;
    cur_joint_values(3) = j3;
    cur_joint_values(4) = j4;
    cur_joint_values(5) = j5;
}

void MoveRobot::setCurPos(double x, double y, double z)
{
    current_pos(0) = x;
    current_pos(1) = y;
    current_pos(2) = z;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "irb120_move_robot");
    ros::NodeHandle n;

    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    VectorXd theta(6);
    VectorXd d(6);
    VectorXd a(6);
    VectorXd alpha(6);
    VectorXd cur_pos(3);
    
    cout << "Initializing Home position" << endl;

    //cur_pos(0) = HOME_X;
    //cur_pos(1) = HOME_Y;
    //cur_pos(2) = HOME_Z;

    cout << "Initializing D-H params" << endl;    

    d(0) = 290; d(1)=0; d(2) = 0; d(3) = 302; d(4) = 0; d(5)= 72;
    a(0) = 0; a(1) = 270; a(2) = 70; a(3) = 0; a(4) = 0; a(5) = 0;
    alpha(0) = 90 * (M_PI/180.0); alpha(1) = 0* (M_PI/180.0); alpha(2) = 90* (M_PI/180.0);
    alpha(3) = -90* (M_PI/180.0); alpha(4) = 90* (M_PI/180.0); alpha(5)= 0* (M_PI/180.0);

    MoveRobot obj(theta, d, a, alpha,n, cur_pos);

    //H matrix for making the robot to go to the BGA Workspace center
    MatrixXd H(4,4);
    MatrixXd home(1,6);
    home(0, 0) = M_PI/2;
    home(0, 1) = 0;
    home(0, 2) = 0;
    home(0, 3) = 0;
    home(0, 4) = -M_PI/2;
    home(0, 5) = 0;

    H = obj.getHomogeneous(home, d, a, alpha);


    obj.setCurJA(home(0,0), home(0,1), home(0,2), home(0,3), home(0,4), home(0,5));
    obj.setCurPos(H(0, 3), H(1, 3), H(2, 3));   

    cout << home <<endl;
    cout << H << endl;
    ros::Rate loop_rate(10);
    for(int i =1;i<10;i++)
    {
        obj.moveRobot(home);
        loop_rate.sleep();
    }

    ros::spin();
}


