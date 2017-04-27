#include <quirkd/libdata_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DataControllerNode");
  ros::NodeHandle nh;
  quirkd::DataController dp(nh);
  ros::Rate r(5);

  dp.update();
  while (ros::ok())
  {
    ros::spinOnce();
    dp.update();
    r.sleep();
  }
  ROS_INFO("DataController Node Exited.");
  return 0;
}