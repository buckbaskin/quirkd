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

#include <swri_profiler/profiler.h>

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
  SWRI_PROFILE("UIManager::alertArrayCB");
  ROS_DEBUG("Alert Array CB");
  /*
   * points -> rects
   * For each rectangle at the end of the list, roll back through and combine with first intersect
   * 
   * If it doesn't match, put it at the beginning
   */
  // convert to rectangles
  std::vector<cv::Rect> rectum;
  for (size_t i = 0; i < msg.alerts.size(); i++)
  {
  	cv::Rect r(msg.alerts[i].min_x, msg.alerts[i].min_y, msg.alerts[i].max_x - msg.alerts[i].min_x, msg.alerts[i].max_y - msg.alerts[i].min_y);
  	rectum.push_back(r);
  }
  // compress rectangles together
  for (size_t i = rectum.size() - 1; i >= 0; i--)
  {
    cv::Rect r = rectum[i];
    for (size_t j = i - 1; i >= 0; i--) {
    	if ((r & rectum[j]).area() > 0) {
    		// they two intersect
    		rectum[j] = rectum[j] | r; // merge them into the j
    		rectum.erase(rectum.begin()+i); // remove the item at index i
    		break;
    	}
    }
    /*
     * If no intersection?
     * Then leave the last one at the end and keep going
     */
  }
  // convert to points
  std::vector<geometry_msgs::Point> points;
  for (size_t i = rectum.size() - 1; i >= 0; i--)
  {
  	extendLineList(&points, &rectum[i]);
  }
  ROS_INFO("Alert Array publishLineList %d", (int)points.size());
  publishLineList(&line_pub_, &points);
}
void UIManager::extendLineList(std::vector<geometry_msgs::Point> *points, cv::Rect *msg)
{
  float min_x = msg->x;
  float min_y = msg->y;
  float max_x = msg->x + msg->width;
  float max_y = msg->y + msg->height;

  if (max_x - min_x < 0.1)
  {
    max_x += 0.05;
    min_x -= 0.05;
  }
  if (max_y - min_y < 0.1)
  {
    max_y += 0.05;
    min_y -= 0.05;
  }
  geometry_msgs::Point minmin;
  minmin.x = min_x;
  minmin.y = min_y;
  geometry_msgs::Point minmax;
  minmax.x = min_x;
  minmax.y = max_y;
  geometry_msgs::Point maxmax;
  maxmax.x = max_x;
  maxmax.y = max_y;
  geometry_msgs::Point maxmin;
  maxmin.x = max_x;
  maxmin.y = min_y;

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
