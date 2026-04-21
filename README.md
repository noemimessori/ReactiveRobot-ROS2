Implementation of two Simple Reactive Robots in Flatland using ROS2
-------------------------------------------------------------------

This project demonstrates a two-robot reactive control system designed to autonomously follow walls in a simulated environment. We chose to use the Robot Operating System (ROS2) and the 2D Flatland simulation environment internally.  
These two differentially driven robots have reactive behaviors and move in a space with a wall to follow inside it. 
One robot is programmed to follow walls, while the other robot dynamically follows the wall or the first one, whichever is closer.

Directory Structure of the Two Robots File
------------------------------------------
Within the project there are two different branches: the first concerns the collision: in NewSensorCollision, the second deals with the round with Roundtriptime. 
├── images/
├── launch/
│   ├── serp_controller.launch.py         # Launch file to start the simulation and robots
├── param/
├── resource/
├── rviz/
├── serp_controller/                      # Robot controller files one and two
│   ├── __init__copy.py                   
│   ├── __init__copy_intermediate.py               
│   ├── __init__initial.py                
│   ├── __init__.py                       # Controller specific to the robot with wall-following behavior and robot-following behavior 
│   ├── second_controller.py              # Controller specific to the robot with only wall-following behavior
│   └── second_controller_initial.py             
├── test/
├── world/                                # Configuration files for environment
├── package.xml
├── setup.cfg
├── setup.py
└── README.txt                            # General project information and usage

Requirements
------------
To run this project, the following software and libraries are required:
ROS2 
Python 
Flatland simulator (compatible with ROS2)

Other Dependencies:
- ROS2 Packages:
  - rclpy: ROS2 Python client library
  - geometry_msgs: ex: Twist, PoseStamped, TwistStamped
- Python Libraries:
  - numpy (for numerical options)
  - math (included with Python)


Installation Instructions
-------------------------
Install Dependencies: Check if ROS2 and any additional dependencies are installed.

Download and Extract the ZIP files:
There are two .zip files, each corresponding to a different branch of the project.
Unzip each file into separate directories and to execute each of them, go to your <ros2_workspace>/src folder and copy the folder in this directory:
extract for example the first one: NewSensorForCollision.zip → NewSensorForCollision/
It's important that not both of the files are in the same time in the <ros2_workspace>/src, otherwise there will be two serp_controller inside the <ros2_workspace>/src

from <ros2_workspace> Build the project and install dependencies:
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --packages-select serp_controller
	source install/setup.bash

Run the package:
	ros2 launch serp_controller serp_controller.launch.py

Do the same procedure with the other folder Roundtriptime.zip → Roundtriptime/
