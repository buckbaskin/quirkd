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
#include <quirkd/ui_manager.h>

namespace quirkd
{
UIManager::UIManager(ros::NodeHandle nh) : n_(nh)
{
  line_pub_ = n_.advertise<visualization_msgs::Marker>("/quirkd/" + ros::this_node::getName() + "/max_line_alert", 1);
  alertArray_sub_ = n_.subscribe("/quirkd/alert_array/notification", 1, &UIManager::alertArrayCB, this);
  ROS_INFO("Done with UIManager constructor");
}
void UIManager::alertArrayCB(const quirkd::AlertArray &msg)
{
  ROS_DEBUG("Alert Array CB");
  std::vector<geometry_msgs::Point> points;
  for (size_t i = 0; i < msg.alerts.size(); i++)
  {
    quirkd::Alert alert = msg.alerts[i];
    extendLineList(&points, &alert);
  }
  ROS_INFO("Alert Array publishLineList %d", (int)points.size());
  publishLineList(&line_pub_, &points);
}
void UIManager::extendLineList(std::vector<geometry_msgs::Point> *points, quirkd::Alert *msg)
{
  if (msg->max_x - msg->min_x < 0.1)
  {
    msg->max_x += 0.05;
    msg->min_x -= 0.05;
  }
  if (msg->max_y - msg->min_y < 0.1)
  {
    msg->max_y += 0.05;
    msg->min_y -= 0.05;
  }
  geometry_msgs::Point minmin;
  minmin.x = msg->min_x;
  minmin.y = msg->min_y;
  geometry_msgs::Point minmax;
  minmax.x = msg->min_x;
  minmax.y = msg->max_y;
  geometry_msgs::Point maxmax;
  maxmax.x = msg->max_x;
  maxmax.y = msg->max_y;
  geometry_msgs::Point maxmin;
  maxmin.x = msg->max_x;
  maxmin.y = msg->min_y;

  points->push_back(minmin);
  points->push_back(minmax);

  points->push_back(minmax);
  points->push_back(maxmax);

  points->push_back(maxmax);
  points->push_back(maxmin);

  points->push_back(maxmin);
  points->push_back(minmin);

  ROS_INFO("Publish lines %.2f %.2f %.2f %.2f", minmin.x, minmin.y, maxmax.x, maxmax.y);
}
void UIManager::publishLineList(ros::Publisher *pub, std::vector<geometry_msgs::Point> *points)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.id = 1;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.type = visualization_msgs::Marker::LINE_LIST;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.05;
  // Chance color based on level
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;

  marker.frame_locked = false;

  marker.points = *points;
  pub->publish(marker);
}
}  // namespace quirkd
int main(int argc, char **argv)
{
  ros::init(argc, argv, "UIManager");
  ROS_INFO("Init in UIManager");
  ros::NodeHandle nh;
  quirkd::UIManager ui(nh);

  ros::spin();
}
