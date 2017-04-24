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

#include <nav_msgs/GetMap.h>
#include <sensor_msgs/image_encodings.h>

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
}  // namespace quirkd
