#include <quirkd/libtime_delay_map.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TimeDelayMapNode");
  ros::NodeHandle nh;
  quirkd::TimeDelayMap tdm(nh);
  tdm.run();
  return 0;
}