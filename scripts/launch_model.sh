#!/bin/bash
roslaunch quirkd quirkd_model.launch &
rosrun --debug gmapping slam_gmapping scan:=base_scan map:=/dynamic_map &&
wait %1 %2
echo Done
