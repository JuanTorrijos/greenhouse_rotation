# greenhouse_rotation
# Automatic-row-switching-algorithm-using-2D-Lidar 

This repository contains the ROS package developed for the research project ‘Automatic row switching algorithm using 2D Lidar’ made by Enrico Mendez and Juan Manuel Torrijos.  

 

This package was built for ROS Noetic and run in Linux Ubuntu 20.04. 

 

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

## Files descriptions  
