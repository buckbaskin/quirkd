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

#include <quirkd/libimage_processing.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <quirkd/Alert.h>
#include <quirkd/AlertArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace quirkd
{
class DataController
{
public:
  DataController(ros::NodeHandle nh);
  ~DataController();
  void laserScanCB(const sensor_msgs::LaserScan msg);
  void update();

private:
  ros::NodeHandle n_;
  ros::Publisher alert_pub_;
  ros::Subscriber laser_sub_;
  ros::ServiceClient dynamic_map_client_;
  ros::ServiceClient static_map_client_;

  image_transport::ImageTransport it_;
  image_transport::Publisher visualization_pub_;

  sensor_msgs::LaserScan last_data;
  tf::StampedTransform last_tf;
  tf::TransformListener tf_;

  void updateAlertPerimeter(quirkd::Alert* alert, const sensor_msgs::LaserScan scan, const tf::StampedTransform tf);
  void alertToRect(quirkd::Alert* alert, cv::Rect* r);
};

}  // namespace quirkd
#endif  // QUIRKD_DATA_CONTROLLER_H