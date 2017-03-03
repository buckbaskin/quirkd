roslaunch quirkd quirkd_model.launch &
rosrun --debug gmapping slam_gmapping scan:=base_scan map:=gmap &&
wait %1 %2
echo Done
