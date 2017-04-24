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
#ifndef QUIRKD_DATA_CONTROLLER_H
#define QUIRKD_DATA_CONTROLLER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
class SemiStaticMap
{
public:
  SemiStaticMap(ros::NodeHandle nh);
  ~SemiStaticMap();
  void run();
  bool setMapCallback(nav_msgs::SetMap::Request& req, nav_msgs::SetMap::Response& res);
  bool updateMapCallback(quirkd::UpdateMap::Request& req, quirkd::UpdateMap::Response& res);
  bool getMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

private:
  ros::NodeHandle n_;
  ros::ServiceClient static_map_client_;
  nav_msgs::OccupancyGrid map_;
  geometry_msgs::PoseWithCovarianceStamped initial_pose_;
}; // class SemiStaticMap
}  // namespace quirkd
#endif  // QUIRKD_DATA_CONTROLLER_H