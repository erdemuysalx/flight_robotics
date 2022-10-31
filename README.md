# Flight Robotics

Repository for ROS-native robot for the Flight Robotics course at University of Stuttgart

## Requirements
* ROS Noetic
* MATLAB/Simulink R2020b
	- MATLAB
	- Simulink
	- MATLAB CODER
	- MATLAB COMPILER
	- MATLAB COMPILER SDK
	- Simulink Coder
	- Embedded Coder
	- Aerospace Blockset
	- ROS Toolbox
	- Aerospace Toolbox
	- Control System Toolbox
	- Symbolic Math Toolbox

## Installation

Execute the following commands in a terminal window:

```
cd ~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/erdemuysalx/flight_robotics.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

The last command needs to be executed in each terminal window in which you would like to run any ROS command, such as "roslaunch", "rostopic", etc.

## Run the Simulation

### Launch Gazebo/ROS simulation

In a terminal window run:

```
source ~/catkin_ws/devel/setup.bash
roslaunch rrbot_gazebo rrbot_world.launch
```

### Launch Simulink Simulation

* Open MATLAB by clicking on the icon in the program list or by typing `matlab-desktop` in a terminal window.
* In the file explorer in MATLAB, navigate into the folder `~/catkin_ws/src/flight_robotics/simulink`.
* Open the file *init.m* and run it by clicking on the green play button that you can find in the Editor tab in MATLAB.
* Type *simulink* in the command window in MATLAB.
* In the simulink window open the file *larix_simulink_gazebo.slx*.
* Run the Simulink simulation by clicking on the small play button in the simulink window.
* A double click on the *remote control* block opens a virtual remote controller that can be used to send commands to the copter.
