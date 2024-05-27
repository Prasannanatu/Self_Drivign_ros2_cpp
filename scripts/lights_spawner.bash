#!/bin/bash
### This bash script is going to spawn Traffic Lights into your simulation at origin 
### Inside the traffic stand

# Source the ROS 2 setup file
source /opt/ros/humble/setup.bash  # Adjust the ROS 2 distribution if necessary

# Source the workspace setup file
source /home/prsnna/ros2_ws/install/local_setup.bash  # Adjust the path according to your workspace

# Define the paths to the SDF files
green_light_sdf=$HOME/.gazebo/models/green_light/model.sdf
red_light_sdf=$HOME/.gazebo/models/red_light/model.sdf
yellow_light_sdf=$HOME/.gazebo/models/yellow_light/model.sdf

# Spawn the traffic lights
ros2 run self_driving_cpp spawning_node $red_light_sdf red_light 
sleep 7.5
ros2 run self_driving_cpp spawning_node $yellow_light_sdf yellow_light 
sleep 1
ros2 run self_driving_cpp spawning_node $green_light_sdf green_Light 
