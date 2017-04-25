#!/bin/bash
ls -d src/*.cpp include/**/*.h srv/*.srv msg/*.msg CMakeLists.txt package.xml | entr -s 'cd ~/ros_ws/ && catkin_make && catkin_make && cd ~/ros_ws/src/quirkd && spd-say "You should know, Build Complete."'

