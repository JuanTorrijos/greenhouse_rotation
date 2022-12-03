# greenhouse_rotation
# Automatic-row-switching-algorithm-using-2D-Lidar 

This repository contains the ROS package developed for the research project ‘Automatic row switching algorithm using 2D Lidar’ made by Enrico Mendez and Juan Manuel Torrijos.  

 

This package was built for ROS Noetic and run in Linux Ubuntu 20.04. 

## Project description

The research project runs an algorithm to make a switching between crop rows in greenhouse based on obstacle avoidance with 2D LiDAR, the algorithm takes LiDAR lectures, filter them to have only the frontal ranges and calculates a free point by adding range vectors considering a number of lectures that have a specific weight depending on what turn will be executed. Once this free point is calculated, it is converted on control references to make the robot to navigate and make a row switching. 
 

## Software used 

Python 3 

Gazeebo 

Rviz 

ROS Noetic 

ROS packages used 

Laser_filters : can be found in http://wiki.ros.org/laser_filters 

Rplidar : can be found in http://wiki.ros.org/rplidar 

## Simulation 

The simulation was made with a simulation of a differential drive robot, jackal. Nevertheless, it can be tested with any robot with a LIDAR.  

To simulate Jackal, follow instructions from the manufacturer in: http://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html  

The Jackal must be equipped with a Lidar. 

To run the simulation: 

Launch the launch file <start_test.launch>  this will open a world in gazebo with walls that will simulate a greenhouse structure, will open a rviz window with the laser scan data and will run the lidar filter node.  

Then run <left.py> or <right.py> node, this is the navigation algorithm each of those runs the program that turns to one side, the one on the name. 

## Algorithm implementation 

To use the algorithm to navigate a Jackal some additional steps are needed. 

Run the <lidar_init.launch> file, this will initialize the Rplidar A3, if another lidar model is being used, it must be initialized. 

Run the <lidar_filter.launch> file, this will filter the lidar samples. 

Then run <left.py> or <right.py> node, this is the navigation algorithm each of those runs the program that turns to one side, the one on the name. 

## Folders descriptions  

### Launch 

This folder contains the launch files of the package 

### Scripts 

This folder contains the ROS nodes. 

### Config 

This folder contains the rviz configurations needed as well as the lidar filter configuration. 

### URDF  

This folder contains some changes made to the jackal simulation. 

### Worlds 

This folder contains the gazeebo world used in the simulation. 

## Additional material  

In this drive folder ROS bags from the experiments can be found in:  https://drive.google.com/drive/folders/140ymzCOkjOyiiQXhW9bEzJSy3rdIP7RN?usp=sharing 
