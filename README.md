# Implementation of two Simple Reactive Robots in Flatland using ROS2

This is **Assignment 1** for the Intelligent Robotics course project. 
- **Reference:** [Course Link](https://sigarra.up.pt/feup/en/ucurr_geral.ficha_uc_view?pv_ocorrencia_id=540794)
- **Paper:** An accompanying paper detailing the project is available in the rticle folder.
- **Video Demo:** A demonstration video is available.

## Group Participants
- Dominik Schneider
- Noemi Messori
- Vicente Lora

---

## Project Overview
This project demonstrates a two-robot reactive control system designed to autonomously follow walls in a simulated environment. We chose to use the Robot Operating System (ROS2) and the 2D Flatland simulation environment internally.  
These two differentially driven robots have reactive behaviors and move in a space with a wall to follow inside it. 
One robot is programmed to follow walls, while the other robot dynamically follows the wall or the first one, whichever is closer.

## Directory Structure
Within the project there are two different branches: the first concerns the collision (NewSensorCollision), the second deals with the round with Roundtriptime. 

`	ext
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
└── README.md                             # General project information and usage
`

## ⚙️ Requirements
To run this project, the following software and libraries are required:
- **ROS2** (Robot Operating System 2)
- **Python** 
- **Flatland Simulator** (compatible with ROS2)

### Other Dependencies:
- **ROS2 Packages:**
  - 
clpy: ROS2 Python client library
  - geometry_msgs: (e.g., Twist, PoseStamped, TwistStamped)
- **Python Libraries:**
  - 
umpy (for numerical options)
  - math (included with Python)

## 🚀 Installation & Build Instructions

### 1. Extract the Project
There are two .zip files, each corresponding to a different branch of the project (NewSensorForCollision.zip and Roundtriptime.zip). Run them one at a time.
1. Unzip the file and move it into your ROS2 workspace src folder (e.g., <ros2_workspace>/src/NewSensorForCollision).

> ** Important:** Make sure only ONE of the project folders is in your <ros2_workspace>/src at a time. If both are present, there will be two serp_controller packages causing conflicts.

### 2. Build the Project
From your <ros2_workspace> directory, install dependencies and build:
`ash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select serp_controller
source install/setup.bash
`

### 3. Run the Package
Launch the simulation and the robots:
`ash
ros2 launch serp_controller serp_controller.launch.py
`

> **Note:** Repeat the same procedure to test the other branch (e.g., extracting and running Roundtriptime.zip -> Roundtriptime/).
