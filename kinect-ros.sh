#!/bin/bash
gnome-terminal -x bash -c "rosrun kinect kinect_socket"
gnome-terminal -x bash -c "rosrun kinect kinect_yolo"
gnome-terminal -x bash -c "rosrun kinect kinect_follow"
