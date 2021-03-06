# dynamics\_ws

WPI RBE-501 S17 Robot Dynamics Project:

Team

* Ethan Barrieau
* Tri Khuu
* Nicholas Longworth
* Jeff Sterniak
* Zhenyu Wan
* Yicong Xu

Project forked from RBE-501 F16 Automated BGA Placement project

Thanks to

* Ankur Agrawal
* Janani Mohan
* Praneeta Mallela
* Sathya Narayanan

Project Goal : Soldering with the ABB IRB 120 industrial Robot (SARA), specifically a fully automated SOIC component soldering system

This repository contains the code needed to automatically operate SARA, including

* Identification and registration of the PCB and SOIC14 chip

* Attachment and operation of custom end effectors
1. Solder paste application syringe
2. IC suction cup
3. Hot air pencil

The packages are as follows: 

irb120\_mover: Master robot control

irb120\_perception: Image Processing for determination of the position and orientation of the SOIC 

irb120\_tf\_calc: Produces the robot TF chain

irb120\_actuator: Sends serial commands to an Arduino for relay actuation

irb120\_cam\_cali: Provides a distortion corrected image from the webcam+loupe

irb120\_vision: Performs PCB location and registration

To operate,

```
roscore
roslaunch abb_irb120_moveit_config moveit_planning_execution.launch
roslaunch irb120_vision lifecam.launch
rosrun irb120_cam_cali irb120_cam_cali_node
rosrun irb120_perception irb120_perception_node
rosrun irb120_vision irb120_vision
rosrun irb120_mover irb120_main_loop
rosrun irb120_mover irb120_robot_mover
```

Dependent Packages to be installed:

ros-indigo: http://www.ros.org/

gazebo2/ gazebo7: http://gazebosim.org/

OpenCV: http://opencv.org/

Eigen: http://eigen.tuxfamily.org/index.php?title=Main\_Page

Serial: http://wjwwood.io/serial/  

ros-controllers: sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
 
ros-gazebo-pkgs for vaccum gripper plugins: https://github.com/ros-simulation/gazebo\_ros\_pkgs

Clone this repository using: 
git clone https://github.com/jsterniak/dynamics\_ws.git

Build the Package using:
catkin\_make

Source the package to ROS Path: 
source devel/setup.bash
