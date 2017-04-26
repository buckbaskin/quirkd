/*
 * Copyright 2017 Buck Baskin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#ifndef QUIRKD_TIME_DELAY_MAP_H
#define QUIRKD_TIME_DELAY_MAP_H

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/SetMap.h>
#include <quirkd/Alert.h>
#include <quirkd/AlertArray.h>
#include <quirkd/UpdateMap.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace quirkd
{
class TimeDelayMap
{
public:
  TimeDelayMap(ros::NodeHandle nh);
  ~TimeDelayMap();
  void run(); // collect 4 maps a sec, return the one that is 1 sec old
  bool getMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

private:
  ros::NodeHandle n_;
  ros::ServiceClient static_map_client_;
  ros::ServiceServer get_map_server_;
  nav_msgs::OccupancyGrid map_;
  std::deque<nav_msgs::OccupancyGrid> map_queue_;
  geometry_msgs::PoseWithCovarianceStamped initial_pose_;
}; // class TimeDelayMap
}  // namespace quirkd
#endif  // QUIRKD_TIME_DELAY_MAP_H