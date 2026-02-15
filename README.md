# UR3 Robot
GUI Implemented to move the robot

## Getting Started

CLone the repo into your workspace 

### Prerequisites

The following packages was tested using the following:

Ubuntu 22.04  
ROS2 Humble   

Make sure you have the following packages and dependencies installed or install them using these commands:

```bash
sudo apt-get install ros-humble-ur
```
NB: other dependencies may be required to be installed according to your ros packages installed.
Refer to this [link](https://docs.ros.org/en/rolling/p/ur_robot_driver/installation/installation.html) if you need any further details 

### Compiling and Folder Structure

The directory UR3_GUI contains the GUI_app folder contains the GUI buttons which controls the robot.

    .
    ├── GUI_app                  # folder contiaining program for controlling the robot
    ├── UR3_ws            	 # the workspace which contains the simulation environment and nodes
    └── README.md                # Documentaion of the subbmision

compile the UR3_GUI using colcon_build.

```
cd ~/UR3_ws/
colcon_build
```

and then source them. 

```
cd ..
source install/setup.bash
```

## Environment Setup

After sourcing your environment to run the robot environment:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.56.101 use_fake_hardware:=true launch_rviz:=false
```
Then run this to see robot simulation enviroment in RVIZ
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```
## Parameters adjustment 

To adjust robot parameters such as the rectangle size or the circle center and radius
do the following:

### Rectangle adjustment
move to the following directory UR3_ws/my_moveit2_planner/src/rectangle_draw.cpp

and adjust the following variables to determine rectangle dimentions.
(NB. rectangle dimentions has a max value of 4.5 meters)

The robot has the following X&Y limits at Z = 0.1m (10cm)

    robot x positive limits 0.125<x<0.45
    robot x negative limits -0.125<x<-0.45

    robot y positive limits 0.125<y<0.125
    robot y negative limits -0.125<y<-0.45

Start by setting rectangle home position which can be found in line 49,50,51
```code
  target_pose1.position.x = 0.25;
  target_pose1.position.y = -0.1;
  target_pose1.position.z = 0.15;
```
And rectangle is being drawn by setting rectangle width and length,  
Set parameter in lines 8,9
```
rectangle_width = 0.2;
rectangle_length = 0.3; 
```

Make sure rectangle (home position_x + width) or (home position_y + length) does not exceed 0.45m  
```
e.g.
0.25 + 0.2 = 0.45
(home position x) +  (rectangle_width) < 0.45m

0.15 + 0.3 = 0.45
(home position y) +  (rectangle_length) < 0.45m
```

### Circle adjustment
move to the following directory:
```
UR3_GUI/my_moveit2_planner/src/circle_draw.cpp
```

The robot has the following X & Y limits at Z = 0.1 m (10cm)

    robot x positive limits 0.125<x<0.45
    robot x negative limits -0.125>x>-0.45

    robot y positive limits 0.125<y<0.125
    robot y negative limits -0.125>y>-0.45

Set the robot centre x and y and the robot radius at lines 7,8,9 where the robot radius + x and y centre does not exceed 0.45/-0.45 also there is a check on the code where it checks for such a condition.

```
double x_center_h = 0.28; //(x)
double y_center_k = 0.35;  //(y)
double radius_r = 0.02;  //radius
``` 

## Moving robot using GUI

Run the GUI to start controlling the robot

Navigate to:
```
Workspaces/GUI_app/qt_proj/build/GUI_app
```

open the following file GUI_app and you will have 2 buttons select them and the robot should move in rectangle or circles accoding to the clicked button



