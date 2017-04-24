#include <quirkd/libsemi_static_map.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SemiStaticMapNode");
  ros::NodeHandle nh;
  quirkd::SemiStaticMap ssm(nh);
  ros::Rate r(30);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  ROS_INFO("SemiStaticMap Node Exited.");
  return 0;
}