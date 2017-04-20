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

namespace quirkd
{
DataController::DataController(ros::NodeHandle nh) : n_(nh), it_(n_)
{
  alert_pub_ = n_.advertise<quirkd::AlertArray>("/quirkd/alert_array/notification", 1);
  laser_sub_ = n_.subscribe("/base_scan", 1, &DataController::laserScanCB, this);
  ROS_INFO("WaitForService(\"static_map\");");
  ros::service::waitForService("static_map");
  static_map_client_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
  ROS_INFO("WaitForService(\"dynamic_map\");");
  ros::service::waitForService("dynamic_map");
  dynamic_map_client_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");
  static_image_pub_ = it_.advertise("/quirkd/test/static_image", 1);
  dynamic_image_pub_ = it_.advertise("/quirkd/test/dynamic_image", 1);
  visualization_pub_ = it_.advertise("/quirkd/test/visualization", 1);
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
  quirkd::Alert alert;
  alert.level = 0;
  alert.min_x = 0;
  alert.max_x = 1;
  alert.min_y = 0;
  alert.max_y = 1;
  updateAlertPerimeter(&alert, last_data, last_tf);

  ROS_INFO("Update Data Processor");
  nav_msgs::GetMap srv;
  cv_bridge::CvImagePtr static_image;
  cv_bridge::CvImagePtr dynamic_image;
  if (static_map_client_.call(srv))
  {
    ROS_DEBUG("Successfull call static map");
    nav_msgs::OccupancyGrid og = srv.response.map;
    static_image = quirkd::imagep::gridToCroppedCvImage(&og, &alert);

    static_image_pub_.publish(static_image->toImageMsg());
  }
  else
  {
    ROS_WARN("Failed to get static map");
  }
  if (dynamic_map_client_.call(srv))
  {
    ROS_DEBUG("Successfull call dynamic map");
    nav_msgs::OccupancyGrid og = srv.response.map;
    dynamic_image = quirkd::imagep::gridToCroppedCvImage(&og, &alert);

    dynamic_image_pub_.publish(dynamic_image->toImageMsg());
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
  aa.alerts = measureDifference(*static_image, *dynamic_image, &alert);

  ROS_INFO("PUBLISHING %d alerts", (int)(aa.alerts.size()));
  alert_pub_.publish(aa);
}
void DataController::updateAlertPerimeter(quirkd::Alert* alert, const sensor_msgs::LaserScan scan,
                                          const tf::StampedTransform tf)
{
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
  double padding = 0.5;
  alert->min_x += -padding;
  alert->min_y += -padding;
  alert->max_x += padding;
  alert->max_y += padding;
}
double DataController::distance_measure(int start_x, int start_y, int end_x, int end_y)
{
  return pow(start_x - end_x, 2) + pow(start_y - end_y, 2);
}
std::vector<quirkd::Alert> DataController::quantifyDifference(cv::Mat* static_processed,
                                                              cv::Mat* dynamic_processed,
                                                              quirkd::Alert* alert)
{
  /* Compute difference
   * Steps:
   * - identify edges
   * - filter out matching edges
   * - look at the size of edges and sum edge differences
   */

  cv::Mat static_edges, dynamic_edges, static_cdst, dynamic_cdst, unmatched_cdst;
  cv::Canny(*static_processed, static_edges, 50, 200, 3);  // TODO check these parameters
  cv::Canny(*dynamic_processed, dynamic_edges, 50, 200, 3);

  cv::cvtColor(static_edges, static_cdst, CV_GRAY2BGR);
  cv::cvtColor(dynamic_edges, dynamic_cdst, CV_GRAY2BGR);
  cv::cvtColor(dynamic_edges, unmatched_cdst, CV_GRAY2BGR);

  // Hough Parameters
  std::vector<cv::Vec4i> static_lines, dynamic_lines;
  const int radius_resolution = 1;
  const double theta_resolution = CV_PI / 180 / 4;
  const int min_line_length = 8;
  const int threshold = min_line_length;         // number of interesections to make a line
  const int max_line_gap = min_line_length - 2;  // largest gap (in pixels) considered on the same line

  cv::HoughLinesP(static_edges, static_lines, radius_resolution, theta_resolution, threshold, min_line_length,
                  max_line_gap);
  for (size_t i = 0; i < static_lines.size(); i++)
  {
    cv::Vec4i l = static_lines[i];
    cv::line(static_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
  }
  cv::HoughLinesP(dynamic_edges, dynamic_lines, radius_resolution, theta_resolution, threshold, min_line_length,
                  max_line_gap);
  for (size_t i = 0; i < dynamic_lines.size(); i++)
  {
    cv::Vec4i l = dynamic_lines[i];
    cv::line(dynamic_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
  }
  double unmatched_quantifier = 0.0;

  // find unmatched dynamic lines (red)
  std::vector<cv::Vec4i> unmatched_dynamic = this->filterEdgesByMatching(static_lines, dynamic_lines);
  for (size_t i = 0; i < unmatched_dynamic.size(); i++)
  {
    cv::Vec4i l = unmatched_dynamic[i];
    cv::line(unmatched_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
    unmatched_quantifier += this->distance_measure(l[0], l[1], l[2], l[3]);
  }
  // find unmatched static lines (red)
  std::vector<cv::Vec4i> unmatched_static = this->filterEdgesByMatching(dynamic_lines, static_lines);
  for (size_t i = 0; i < unmatched_static.size(); i++)
  {
    cv::Vec4i l = unmatched_static[i];
    cv::line(unmatched_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 2, CV_AA);
    unmatched_quantifier += this->distance_measure(l[0], l[1], l[2], l[3]);
  }

  int total_alert_level = (int)(unmatched_quantifier);

  cv_bridge::CvImage anomaly_visualization;
  anomaly_visualization.encoding = sensor_msgs::image_encodings::BGR8;
  anomaly_visualization.image = unmatched_cdst;
  ROS_DEBUG("publish visualization");
  visualization_pub_.publish(anomaly_visualization.toImageMsg());

  // return the minimized set of alerts
  std::vector<quirkd::Alert> alerts = this->minimizeAlerts(unmatched_static, unmatched_dynamic, alert);
  return alerts;
}
std::vector<cv::Vec4i> DataController::filterEdgesByMatching(std::vector<cv::Vec4i> original,
                                                             std::vector<cv::Vec4i> new_edges)
{
  std::vector<cv::Vec4i> unmatched_set;
  const double limit = 1000;
  for (size_t i = 0; i < new_edges.size(); i++)
  {  // for each edge in the new edges
    cv::Vec4i seeker = new_edges[i];
    bool matched = false;
    double min_best = 10000000;
    for (size_t i = 0; i < original.size(); i++)
    {  // check each of the original edges
      cv::Vec4i candidate = original[i];
      // for the beginning and end of the seeker line, check the distance to the candidate endpoints
      // take the orientation with the best overall score
      // if it beats (smaller than) the existing best match, update the params

      // front to front
      double forward_front = this->distance_measure(seeker[0], seeker[1], candidate[0], candidate[1]);
      double forward_back = this->distance_measure(seeker[2], seeker[3], candidate[2], candidate[3]);
      if (forward_front < limit / 2 && forward_back < limit / 2 && forward_back + forward_front < limit / 2)
      {
        matched = true;
        break;
      }
      min_best = std::min(min_best, forward_front + forward_back);
      double reverse_front = this->distance_measure(seeker[0], seeker[1], candidate[2], candidate[3]);
      double reverse_back = this->distance_measure(seeker[2], seeker[3], candidate[0], candidate[1]);
      if (reverse_front < limit / 2 && reverse_back < limit / 2 && reverse_back + reverse_front < limit / 2)
      {
        matched = true;
        break;
      }
      min_best = std::min(min_best, reverse_front + reverse_back);
    }
    // if there isn't a good match, add it to the unmatched set
    if (!matched)
    {
      // ROS_INFO("Best min %f", min_best);
      unmatched_set.push_back(seeker);
    }
    else
    {
      // I found a good match!
    }
  }
  return unmatched_set;
}
std::vector<quirkd::Alert> DataController::measureDifference(cv_bridge::CvImage static_image,
                                                             cv_bridge::CvImage dynamic_image,
                                                             quirkd::Alert* alert)
{
  quirkd::imagep::preprocessImages(&(static_image.image), &(dynamic_image.image), alert);
  std::vector<quirkd::Alert> result = this->quantifyDifference(&(static_image.image), &(dynamic_image.image), alert);
  return result;
}
std::vector<quirkd::Alert> DataController::minimizeAlerts(std::vector<cv::Vec4i> unmatched_static,
                                                          std::vector<cv::Vec4i> unmatched_dynamic,
                                                          quirkd::Alert* alert_msg)
{
  std::vector<quirkd::Alert> alerts;
  float map_resolution = 0.05;  // meters per cell/pixel
  for (size_t i = 0; i < unmatched_static.size(); i++)
  {
    cv::Vec4i l = unmatched_static[i];
    quirkd::Alert alert;
    alert.level = (int)(this->distance_measure(l[0], l[1], l[2], l[3]));
    // ROS_INFO("alert.level %d", alert.level);
    alert.min_x = std::min(l[0], l[2]) * map_resolution + alert_msg->min_x;
    alert.max_x = std::max(l[0], l[2]) * map_resolution + alert_msg->min_x;
    if (alert.max_x - alert.min_x < .01)
    {
      alert.max_x += .05;
      alert.min_x -= .05;
    }
    // ROS_INFO("alert.min_x %.2f", alert.min_x);
    // ROS_INFO("alert.max_x %.2f", alert.max_x);
    alert.min_y = std::min(l[1], l[3]) * map_resolution + alert_msg->min_y;
    alert.max_y = std::max(l[1], l[3]) * map_resolution + alert_msg->min_y;
    if (alert.max_y - alert.min_y < .01)
    {
      alert.max_y += .05;
      alert.min_y -= .05;
    }
    // ROS_INFO("alert.min_y %.2f", alert.min_y);
    // ROS_INFO("alert.max_y %.2f", alert.max_y);
    alerts.push_back(alert);
  }
  for (size_t i = 0; i < unmatched_dynamic.size(); i++)
  {
    cv::Vec4i l = unmatched_dynamic[i];
    quirkd::Alert alert;
    alert.level = (int)(this->distance_measure(l[0], l[1], l[2], l[3]));
    // ROS_INFO("alert.level %d", alert.level);
    alert.min_x = std::min(l[0], l[2]) * map_resolution + alert_msg->min_x;
    alert.max_x = std::max(l[0], l[2]) * map_resolution + alert_msg->min_x;
    if (alert.max_x - alert.min_x < .01)
    {
      alert.max_x += .05;
      alert.min_x -= .05;
    }
    // ROS_INFO("alert.min_x %.2f", alert.min_x);
    // ROS_INFO("alert.max_x %.2f", alert.max_x);
    alert.min_y = std::min(l[1], l[3]) * map_resolution + alert_msg->min_y;
    alert.max_y = std::max(l[1], l[3]) * map_resolution + alert_msg->min_y;
    if (alert.max_y - alert.min_y < .01)
    {
      alert.max_y += .05;
      alert.min_y -= .05;
    }
    // ROS_INFO("alert.min_y %.2f", alert.min_y);
    // ROS_INFO("alert.max_y %.2f", alert.max_y);
    alerts.push_back(alert);
  }
  return alerts;
}

}  // namespace quirkd
