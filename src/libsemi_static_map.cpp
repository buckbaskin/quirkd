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
#include <quirkd/libsemi_static_map.h>

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

namespace quirkd
{
SemiStaticMap::SemiStaticMap(ros::NodeHandle nh) : n_(nh)
{
  ROS_INFO("WaitForService(\"static_map\");");
  ros::service::waitForService("static_map");
  static_map_client_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
}
SemiStaticMap::~SemiStaticMap()
{
  ROS_INFO("Destroying SemiStaticMap");
}
void SemiStaticMap::run()
{
  ROS_INFO("SSM run");
  ros::Rate r(30);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  ROS_INFO("SemiStaticMap Node Exited.");
}
bool SemiStaticMap::setMapCallback(nav_msgs::SetMap::Request& req, nav_msgs::SetMap::Response& res) {
  map_ = req.map;
  initial_pose_ = req.initial_pose;
  res.success = true;
  return true;
}
bool SemiStaticMap::updateMapCallback(quirkd::UpdateMap::Request& req, quirkd::UpdateMap::Response& res) {
  map_ = req.map;
  res.success = mergeMap(&map_, &req.map);
  return res.success;
}
bool SemiStaticMap::getMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {
  res.map = map_;
  return true;
}

bool SemiStaticMap::mergeMap(nav_msgs::OccupancyGrid* original, nav_msgs::OccupancyGrid* new_section) {
  if (original->header.frame_id != new_section->header.frame_id) {
    // TODO? use transforms to align properly?
    // TODO? make sure those transforms are translation only for now
    return false;
  }
  if (original->info.resolution != new_section->info.resolution) {
    // TODO? make more educated checks and or use OpenCV to scale?
    return false;
  }
  if (original->info.origin.orientation.x != new_section->info.origin.orientation.x ||
    original->info.origin.orientation.y != new_section->info.origin.orientation.y ||
    original->info.origin.orientation.z != new_section->info.origin.orientation.z ||
    original->info.origin.orientation.w != new_section->info.origin.orientation.w) {
    // TODO? consider allowing rotational alignments in the future
    return false;
  }
  double common_resolution = original->info.resolution;
  // The maps are in the same frame (map?), the same resolution and oriented in the same direction
  cv::Mat og_mat = mapToMat(original);
  cv::Mat new_mat = mapToMat(new_section);
  cv::Rect og_rect = mapToRect(original);
  cv::Rect new_rect = mapToRect(new_section);

  cv::Rect both_rect = og_rect | new_rect;
  cv::Mat both_mat(
    both_rect.height / common_resolution,
    both_rect.width / common_resolution,
    CV_8SC1,
    cv::Scalar(-1)
    );

  // TODO place the other two maps into both_mat

  matToMap(both_mat, original);
  return true;
}
cv::Mat SemiStaticMap::mapToMat(nav_msgs::OccupancyGrid* map) {
  // TODO implement
  cv::Mat toReturn;
  return toReturn;
}
cv::Rect SemiStaticMap::mapToRect(nav_msgs::OccupancyGrid* map) {
  // TODO implement
  cv::Rect toReturn;
  return toReturn;
}
void SemiStaticMap::matToMap(cv::Mat as_img, nav_msgs::OccupancyGrid* map) {
  // TODO implement

}
}  // namespace quirkd
