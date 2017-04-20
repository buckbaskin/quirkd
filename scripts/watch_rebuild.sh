#!/bin/bash
ls -d src/* include/quirkd/* CMakeLists.txt | entr -s 'cd ~/ros_ws/ && catkin_make && catkin_make && cd ~/ros_ws/src/quirkd'

