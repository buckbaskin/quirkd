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

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <quirkd/Alert.h>
#include <quirkd/AlertArray.h>
#include <visualization_msgs/Marker.h>

namespace quirkd {
  UIManager::UIManager(ros::NodeHandle nh) : n_(nh)
  {
    low_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("/low_alert", 1);
    warn_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("/warn_alert", 1);
    max_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("/max_alert", 1);
    line_pub_ = n_.advertise<visualization_msgs::Marker>("/max_line_alert", 1);
    alert_sub_ = n_.subscribe("/quirkd/alert/notification", 1, &UIManager::alertCB, this);
    alertArray_sub_ = n_.subscribe("/quirkd/alert_array/notification", 1, &UIManager::alertArrayCB, this);
    ROS_INFO("Done with UIManager constructor");
  }
  void UIManager::alertCB(const quirkd::Alert &alert)
  {
    ROS_DEBUG("Alert CB");
    if (alert.level < 10)
    {
      publishPolygon(&low_pub_, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
    }
    else if (alert.level > 100)
    {
      publishPolygon(&max_pub_, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
    }
    else
    {
      publishPolygon(&warn_pub_, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
    }
  }
  void UIManager::alertArrayCB(const quirkd::AlertArray &msg)
  {
    ROS_DEBUG("Alerts CB");
    std::vector<geometry_msgs::Point> points;
    for (size_t i = 0; i < msg.alerts.size(); i++)
    {
      quirkd::Alert alert = msg.alerts[i];
      extendLineList(&points, &alert);
    }
    publishLineList(&max_pub_, &points);
  }
  void UIManager::extendLineList(std::vector<geometry_msgs::Point> *points, quirkd::Alert *msg)
  {
    geometry_msgs::Point minmin;
    minmin.x = msg->min_x;
    minmin.y = msg->min_y;
    geometry_msgs::Point minmax;
    minmin.x = msg->min_x;
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
  }
  void UIManager::publishLineList(ros::Publisher *pub, std::vector<geometry_msgs::Point> *points)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.id = 1;
    marker.action = visualization_msgs::Marker::LINE_LIST;
    marker.type = visualization_msgs::Marker::MODIFY;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    // Chance color based on level
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.frame_locked = false;

    marker.points = *points;
    pub->publish(marker);
  }
  void UIManager::publishPolygon(ros::Publisher *pub, float min_x, float max_x, float min_y, float max_y)
  {
    if (max_x - min_x < .01)
    {
      max_x += .05;
      min_x -= .05;
    }
    if (max_y - min_y < .01)
    {
      max_y += .05;
      min_y -= .05;
    }
    geometry_msgs::PolygonStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "map";

    geometry_msgs::Point32 p1;
    p1.x = min_x;
    p1.y = min_y;
    p1.z = 0;
    ps.polygon.points.push_back(p1);
    geometry_msgs::Point32 p2;
    p2.x = max_x;
    p2.y = min_y;
    p2.z = 0;
    ps.polygon.points.push_back(p2);
    geometry_msgs::Point32 p3;
    p3.x = max_x;
    p3.y = max_y;
    p3.z = 0;
    ps.polygon.points.push_back(p3);
    geometry_msgs::Point32 p4;
    p4.x = min_x;
    p4.y = max_y;
    p4.z = 0;
    ps.polygon.points.push_back(p4);

    ROS_INFO("Publish polygon %.2f %.2f %.2f %.2f", min_x, max_x, min_y, max_y);
    pub->publish(ps);
  }
} // namespace quirkd
int main(int argc, char **argv)
{
  ros::init(argc, argv, "UIManager");
  ROS_INFO("Init in UIManager");
  ros::NodeHandle nh;
  quirkd::UIManager ui(nh);

  ros::spin();
}
