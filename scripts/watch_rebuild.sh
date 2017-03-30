#!/bin/bash
ls -d src/* | entr -s 'cd ~/ros_ws/ && catkin_make && catkin_make && cd ~/ros_ws/src/quirkd'

