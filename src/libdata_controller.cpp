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
#include <quirkd/libdata_controller.h>

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include <nav_msgs/GetMap.h>
#include <sensor_msgs/image_encodings.h>
#include <swri_profiler/profiler.h>

namespace quirkd
{
DataController::DataController(ros::NodeHandle nh) : n_(nh), it_(n_)
{
  alert_pub_ = n_.advertise<quirkd::AlertArray>("/quirkd/" + ros::this_node::getName() + "/alert_array", 1);
  laser_sub_ = n_.subscribe("/base_scan", 1, &DataController::laserScanCB, this);
  ROS_INFO("WaitForService(\"static_map\");");
  ros::service::waitForService("static_map");
  static_map_client_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
  ROS_INFO("WaitForService(\"dynamic_map\");");
  ros::service::waitForService("dynamic_map");
  dynamic_map_client_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");
  visualization_pub_ = it_.advertise("/quirkd/" + ros::this_node::getName() + "/visualization", 1);
}
DataController::~DataController()
{
  ROS_INFO("Destroying DataController");
}
void DataController::laserScanCB(const sensor_msgs::LaserScan msg)
{
  ROS_INFO("Laser Scan Callback");
  last_data = msg;
  try
  {
    ROS_INFO("Waiting for transform");
    tf_.waitForTransform("/map", "/base_laser_link", ros::Time::now(), ros::Duration(3.0));
    tf_.lookupTransform("/map", "/base_laser_link", ros::Time::now(), last_tf);
    ROS_INFO("tf success for /map to /base_laser_link");
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("tf fetch failed. %s", ex.what());
  }
}
void DataController::update()
{
  SWRI_PROFILE("DataController::update");
  quirkd::Alert alert;
  alert.level = 0;
  alert.min_x = 0;
  alert.max_x = 1;
  alert.min_y = 0;
  alert.max_y = 1;
  updateAlertPerimeter(&alert, last_data, last_tf);
  cv::Rect alert_rect;
  alertToRect(&alert, &alert_rect);

  ROS_INFO("Update Data Processor");
  nav_msgs::GetMap srv;
  cv_bridge::CvImagePtr static_image;
  cv_bridge::CvImagePtr dynamic_image;
  if (static_map_client_.call(srv))
  {
    SWRI_PROFILE("static map call");
    ROS_DEBUG("Successfull call static map");
    nav_msgs::OccupancyGrid og = srv.response.map;
    static_image = quirkd::imagep::gridToCroppedCvImage(&og, &alert_rect);
  }
  else
  {
    ROS_WARN("Failed to get static map");
  }
  if (dynamic_map_client_.call(srv))
  {
    SWRI_PROFILE("dynamic map call");
    ROS_DEBUG("Successfull call dynamic map");
    nav_msgs::OccupancyGrid og = srv.response.map;
    dynamic_image = quirkd::imagep::gridToCroppedCvImage(&og, &alert_rect);
  }
  else
  {
    ROS_WARN("Failed to get dynamic map");
    dynamic_image = static_image;
  }

  // two images (static (base) image and dynamic image)
  // perimeter encoded as part of the alert and images aligned to alert perimeter

  // use a single method that captures preprocessing and quantification
  quirkd::AlertArray aa;
  aa.alerts = quirkd::imagep::measureDifference(*static_image, *dynamic_image, &alert, &visualization_pub_);

  ROS_INFO("PUBLISHING %d alerts", (int)(aa.alerts.size()));
  alert_pub_.publish(aa);
}
void DataController::alertToRect(quirkd::Alert* alert, cv::Rect* r)
{
  alert->min_x;
  alert->max_x;
  alert->min_y;
  alert->max_y;
  // toReturn(x, y, width, height);
  r->x = alert->min_x;
  r->y = alert->min_y;
  r->width = alert->max_x - alert->min_x;
  r->height = alert->max_y - alert->min_y;
}
void DataController::updateAlertPerimeter(quirkd::Alert* alert, const sensor_msgs::LaserScan scan,
                                          const tf::StampedTransform tf)
{
  SWRI_PROFILE("DataController::updateAlertPerimeter");
  double base_x = tf.getOrigin().x();
  double base_y = tf.getOrigin().y();
  double r, p, base_heading;
  tf::Matrix3x3 m(tf.getRotation());
  m.getRPY(r, p, base_heading);

  double scan_min = base_heading + scan.angle_min;
  double scan_inc = scan.angle_increment;
  double heading = scan_min;

  alert->min_x = base_x;
  alert->max_x = base_x;
  alert->min_y = base_y;
  alert->max_y = base_y;

  double live_x, live_y;

  if (true)
  {
    SWRI_PROFILE("for loop to get possible area");
    for (int i = 0; i < scan.ranges.size(); i++)
    {
      double dist = scan.ranges[i];

      live_x = base_x + dist * cos(heading);
      live_y = base_y + dist * sin(heading);
      alert->min_x = std::min(live_x, double(alert->min_x));
      alert->max_x = std::max(live_x, double(alert->max_x));
      alert->min_y = std::min(live_y, double(alert->min_y));
      alert->max_y = std::max(live_y, double(alert->max_y));

      heading += scan_inc;
    }
  }
  double padding = 0.5;
  alert->min_x += -padding;
  alert->min_y += -padding;
  alert->max_x += padding;
  alert->max_y += padding;
}

}  // namespace quirkd
