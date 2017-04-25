#include <quirkd/libsemi_static_map.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "SemiStaticMapNode");
  ros::NodeHandle nh;
  quirkd::SemiStaticMap ssm(nh);
  ssm.run();
  return 0;
}