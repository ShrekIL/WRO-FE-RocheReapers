#!/bin/bash
gnome-terminal \
--tab -e "zsh -c 'source $HOME/.zshrc;sudo systemctl stop start_app_node.service;ros2 launch slam slam.launch.py enable_save:=false'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 10;ros2 launch peripherals teleop_key_control.launch.py'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 10;rviz2 rviz2 -d /home/ubuntu/ros2_ws/src/slam/rviz/slam_desktop.rviz'" \
--tab -e "zsh -c 'source $HOME/.zshrc;sleep 10;ros2 run slam map_save'"
