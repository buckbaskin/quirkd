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
#ifndef QUIRKD_UI_MANAGER_H
#define QUIRKD_UI_MANAGER_H

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <quirkd/Alert.h>
#include <quirkd/AlertArray.h>
#include <visualization_msgs/Marker.h>

namespace quirkd
{
class UIManager
{
public:
  UIManager(ros::NodeHandle nh);
  void alertCB(const quirkd::Alert &alert);
  void alertArrayCB(const quirkd::AlertArray &msg);
  void extendLineList(std::vector<geometry_msgs::Point> *points, cv::Rect *msg);
  void publishLineList(ros::Publisher *pub, std::vector<geometry_msgs::Point> *points);
  void publishPolygon(ros::Publisher *pub, float min_x, float max_x, float min_y, float max_y);

private:
  ros::NodeHandle n_;
  // ros::Publisher low_pub_;
  // ros::Publisher warn_pub_;
  // ros::Publisher max_pub_;
  ros::Publisher line_pub_;
  ros::Subscriber alert_sub_;
  ros::Subscriber alertArray_sub_;
};
}  // namespace quirkd
#endif  // QUIRKD_UI_MANAGER_H