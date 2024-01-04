## SLAM and Navigation

A launch file is added under the "/launch/navigation.launch" directory which executes the "SLAM-gmapping" and "move_base" packages.

The SLAM-gmapping algorithm subscribes to the "/scan" topic and generates a map of the environment and provides the tf "map -> odom" in the TF tree.

The "move_base" package creates cost maps (global and local) using the provided static map and laser scan and planning a path toward the provided goal for the robot.

To start:
```bash
sudo apt install ros-"your-ros-distro"-slam-gmapping
roslaunch slam_test world.launch
roslaunch slam_test navigation.launch
```
note: you need to change the "rviz.rviz" directory in the "/launch/navigation.launch" file 

For the SLAM part, There are other packages developed using different methods and sensors. e.g. Cartographer uses both Lidar and IMU and creates a more accurate map of the environment. However, for a simulation environment, SLAM-gmapping algorithm is accurate enough. 

## Navigating the robot
In order to navigate the robot, you can either:

1. use "2D Nav Goal" in rviz and generate single/multiple goal points for the robot,
2. run
```bash
rosrun slam_test goals
```
There are three arbitrary goal points are defined in the "goals.cpp" 

## Keyboard controller
run 
```bash
rosrun slam_test keyboard.py
```
## Static Obstacle Avoidance achievement
In order to avoid static obstacles, the "move-base" module creates a cost map of the environment which assigns a value (in a range of [0,100]). While planning a path for the robot, it avoids making a path on a cell with a cost higher than a threshhold.

