# Robot Arm Sample Project (ROS 1)

## About

This is a sample ROS1 project for controlling a robot arm using gazebo ros controllers

## Starting

Open in VSCode and use devcontainer (F1 -> Rebuild and Reopen in Container). Before this, prepare the usual devcontainer environment with VSCode and Docker.

Wait for the devconatainer to start. Open another terminal in VSCode and build the project using:
```
catkin_make
```
Source the environment with:
```
source devel/setup.bash
```

## Running 
In one VSCode terminal run the launch file:
```
roslaunch robot_arm arm.launch
```

and in another terminal run the test program:
```
rosrun robot_arm test_controller.py
```