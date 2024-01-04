## Introduction
This is a Gazebo environment with a two-wheel robot called Pioneer.

The Gazebo world and robot URDF are not supposed to be modified.

Please based on the Gazebo simulation, achieve the SLAM for navigation.

**You can use any public ROS package.**

## Launch the Gazebo simulation (ROS 1)
```bash
roslaunch slam_test world.launch
```

## Task 
1. Setup ROS envorinment, and explore Gazebo env
2. Mapping: Generate the map based on SLAM
3. Navigation: Proof the SLAM working by navitating 
    Hint: generate a target point based on Rviz
4. Write the documentation how you approach this project

## Bonus
1. Follow a sequence of target points (not a single target point)
2. Create a keyboard controller
2. Static Obstacle Avoidance achievement
3. Developing based on Docker would be a bonus