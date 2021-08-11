#!/usr/bin/env bash

echo "Installing ROS depedencies..."
sudo apt update
sudo apt install ros-melodic-people-msgs ros-melodic-moveit-ros-planning-interface ros-melodic-four-wheel-steering-msgs ros-melodic-urdf-geometry-parser
echo "Done."
