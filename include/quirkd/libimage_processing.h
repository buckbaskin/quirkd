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
#ifndef QUIRKD_IMAGE_PROCESSING_H
#define QUIRKD_IMAGE_PROCESSING_H

#include <ros/ros.h>

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
namespace imagep
{
cv_bridge::CvImagePtr gridToCroppedCvImage(nav_msgs::OccupancyGrid* grid);
cv_bridge::CvImagePtr gridToCroppedCvImage(nav_msgs::OccupancyGrid* grid, cv::Rect* alert);
void preprocessImages(cv::Mat* static_image, cv::Mat* dynamic_image, quirkd::Alert* alert);
std::vector<cv::Vec4i> filterEdgesByMatching(std::vector<cv::Vec4i> original,
                                             std::vector<cv::Vec4i> new_edges);
std::vector<quirkd::Alert> quantifyDifference(cv::Mat* static_processed,
                                              cv::Mat* dynamic_processed,
                                              quirkd::Alert* alert,
                                              image_transport::Publisher* visualization_pub);
std::vector<quirkd::Alert> measureDifference(cv_bridge::CvImage static_image,
                                             cv_bridge::CvImage dynamic_image,
                                             quirkd::Alert* alert,
                                             image_transport::Publisher* visualization_pub);
double distance_measure(int start_x, int start_y, int end_x, int end_y);
std::vector<quirkd::Alert> minimizeAlerts(std::vector<cv::Vec4i> unmatched_static,
                                                          std::vector<cv::Vec4i> unmatched_dynamic,
                                                          quirkd::Alert* alert_msg);
} // namespace imagep
} // namespace quirkd
#endif // QUIRKD_IMAGE_PROCSESING_H