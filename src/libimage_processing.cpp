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
#include <quirkd/libimage_processing.h>

#include <swri_profiler/profiler.h>

namespace quirkd
{
namespace imagep
{
void cvImageToGrid(cv_bridge::CvImagePtr ptr, nav_msgs::OccupancyGrid* grid)
{
}
cv_bridge::CvImagePtr gridToCvImage(nav_msgs::OccupancyGrid* grid)
{
  nav_msgs::MapMetaData info = grid->info;
  float resolution = info.resolution;  // meters per pixel
  int cell_width = info.width;
  int cell_height = info.height;
  float origin_x = info.origin.position.x;
  float origin_y = info.origin.position.x;
  /*
   * From documentation:
   * The map data in row major order, starting at 0,0.
   * Valid values are in the range [0, 100]
   */
  std::vector<int8_t> data = grid->data;
  std::vector<uint8_t> unsigned_data;
  unsigned_data.resize(data.size());  // allocate and fill vector to match num elements of data

  for (std::vector<int8_t>::size_type i = 0; i < data.size(); i++)
  {
    int8_t old_int = data[i];
    // assume unknown values (signed -1) are just 0
    // assumption is that unseen parts of the map have no obstacles until proven otherwise
    uint8_t new_int = (uint8_t)((old_int == -1) ? 0 : old_int);
    unsigned_data[(std::vector<uint8_t>::size_type)i] = new_int;
  }

  // Create the ROS Image Message

  sensor_msgs::Image converted_image;
  converted_image.width = cell_width;
  converted_image.height = cell_height;
  converted_image.encoding = sensor_msgs::image_encodings::MONO8;

  converted_image.step = cell_width;
  converted_image.data = unsigned_data;

  int map_cell_origin_x = (int)(origin_x / resolution);  // pixels
  int map_cell_origin_y = (int)(origin_y / resolution);  // pixels

  cv::Rect map_region(map_cell_origin_x, map_cell_origin_y, cell_width, cell_height);
  return gridToCroppedCvImage(grid, &map_region);
}
cv_bridge::CvImagePtr gridToCroppedCvImage(nav_msgs::OccupancyGrid* grid, cv::Rect* alert)
{
  // Unpack the Occupancy Grid

  SWRI_PROFILE("imagep::gridToCroppedCvImage");
  nav_msgs::MapMetaData info = grid->info;
  float resolution = info.resolution;  // meters per pixel
  int cell_width = info.width;
  float map_width = cell_width * resolution;
  int cell_height = info.height;
  float map_height = cell_height * resolution;
  // I'm assuming the map isn't rotated right now because that could be really complex
  float origin_x = info.origin.position.x;
  float origin_y = info.origin.position.x;
  /*
   * From documentation:
   * The map data in row major order, starting at 0,0.
   * Valid values are in the range [0, 100]
   */
  std::vector<int8_t> data = grid->data;
  std::vector<uint8_t> unsigned_data;
  unsigned_data.resize(data.size());  // allocate and fill vector to match num elements of data

  if (true)
  {
    SWRI_PROFILE("resassign data to unsigned floor");
    for (std::vector<int8_t>::size_type i = 0; i < data.size(); i++)
    {
      int8_t old_int = data[i];
      // assume unknown values (signed -1) are just 0
      // assumption is that unseen parts of the map have no obstacles until proven otherwise
      uint8_t new_int = (uint8_t)((old_int == -1) ? 0 : old_int);
      unsigned_data[(std::vector<uint8_t>::size_type)i] = new_int;
    }
  }
  // Create the ROS Image Message

  sensor_msgs::Image converted_image;
  converted_image.width = cell_width;
  converted_image.height = cell_height;
  converted_image.encoding = sensor_msgs::image_encodings::MONO8;

  converted_image.step = cell_width;
  converted_image.data = unsigned_data;

  // Use CV Bridge to get the CvImagePtr

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    SWRI_PROFILE("cv_bridge::toCvCopy");
    cv_ptr = cv_bridge::toCvCopy(converted_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  /*
   * Use the OpenCV cv::Rect intersection operator to find the
   *  relevant intersection of the full image with the base image
   *  Cut this out as a region of interest and paste it over the
   *  black base image
   * Return this image
   */
  int map_cell_origin_x = (int)(origin_x / resolution);  // pixels
  int map_cell_origin_y = (int)(origin_y / resolution);  // pixels

  cv::Rect map_region(map_cell_origin_x, map_cell_origin_y, cell_width, cell_height);
  ROS_DEBUG("map x %d y %d w %d h %d", map_region.x, map_region.y, map_region.width, map_region.height);

  int p_cell_origin_x = (int)(alert->x / resolution);    // pixels
  int p_cell_origin_y = (int)(alert->y / resolution);    // pixels
  int perim_width = (int)(alert->width / resolution);    // pixels
  int perim_height = (int)(alert->height / resolution);  // pixels
  ROS_DEBUG("perim w: %d h: %d", perim_width, perim_height);

  cv::Rect perimeter(p_cell_origin_x, p_cell_origin_y, perim_width, perim_height);
  ROS_DEBUG("perimeter x %d y %d w %d h %d", perimeter.x, perimeter.y, perimeter.width, perimeter.height);

  cv::Rect intersect = map_region & perimeter;

  ROS_DEBUG("I made my map rectangle");

  // shift intersection to the coordinates of the cropped image
  intersect.x -= map_cell_origin_x;
  intersect.y -= map_cell_origin_y;

  ROS_DEBUG("shift intersect 1 x %d y %d w %d h %d", intersect.x, intersect.y, intersect.width, intersect.height);

  // Assertion: 0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols && 0 <= roi.y && 0 <= roi.height && roi.y
  // + roi.height <= m.rows
  ROS_DEBUG("conditions 1: %d %d %d %d %d %d", 0 <= intersect.x, 0 <= intersect.width,
            intersect.x + intersect.width <= cv_ptr->image.cols, 0 <= intersect.y, 0 <= intersect.height,
            intersect.y + intersect.height <= cv_ptr->image.rows);
  // this is the subset of the two images from the map that should be copied into the base
  cv::Mat cropped_map = cv_ptr->image(intersect);

  ROS_DEBUG("cropped map");

  cv::Mat base(perim_height, perim_width, CV_8UC1, cv::Scalar(0));
  // shift intersection to the coordinates of the base image
  intersect.x = intersect.x + map_cell_origin_x - p_cell_origin_x;
  intersect.y = intersect.y + map_cell_origin_y - p_cell_origin_y;

  ROS_DEBUG("shift intersect 2 x %d y %d w %d h %d", intersect.x, intersect.y, intersect.width, intersect.height);
  ROS_DEBUG("base_info x 0 y 0 w %d h %d", base.cols, base.rows);

  // Assertion: 0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols && 0 <= roi.y && 0 <= roi.height && roi.y
  // + roi.height <= m.rows
  ROS_DEBUG("conditions 2: %d %d %d %d %d %d", 0 <= intersect.x, 0 <= intersect.width,
            intersect.x + intersect.width <= base.cols, 0 <= intersect.y, 0 <= intersect.height,
            intersect.y + intersect.height <= base.rows);
  ROS_DEBUG("conditions 2.1:\n0 <= %d\n0 <= %d\n%d <= %d\n0 <= %d\n0 <= %d\n%d <= %d", intersect.x, intersect.width,
            intersect.x + intersect.width, base.cols, 0 <= intersect.y, 0 <= intersect.height,
            intersect.y + intersect.height, base.rows);

  cv::Mat base_roi(base, intersect);

  ROS_DEBUG("shift intersect 2");

  // copy the map overlay to the base image
  if (intersect.area() > 0)
  {
    SWRI_PROFILE("cropped_map.copyTo(base_roi)");
    cropped_map.copyTo(base_roi);
    ROS_DEBUG("Overlay done");
  }
  else
  {
    ROS_DEBUG("No overlay (no area)");
  }

  cv_ptr->image = base;
  return cv_ptr;
}
void preprocessImages(cv::Mat* static_image, cv::Mat* dynamic_image, quirkd::Alert* alert)
{
}
std::vector<quirkd::Alert> quantifyDifference(cv::Mat* static_processed, cv::Mat* dynamic_processed,
                                              quirkd::Alert* alert, image_transport::Publisher* visualization_pub)
{
  /* Compute difference
   * Steps:
   * - identify edges
   * - filter out matching edges
   * - look at the size of edges and sum edge differences
   */

  cv::Mat static_edges, dynamic_edges, static_cdst, dynamic_cdst, unmatched_cdst;
  if (true)
  {
    SWRI_PROFILE("imagep::quantifyDifference");
    SWRI_PROFILE("Canny calls");
    cv::Canny(*static_processed, static_edges, 50, 200, 3);  // TODO check these parameters
    cv::Canny(*dynamic_processed, dynamic_edges, 50, 200, 3);
  }

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
  std::vector<cv::Vec4i> unmatched_dynamic = filterEdgesByMatching(static_lines, dynamic_lines);
  for (size_t i = 0; i < unmatched_dynamic.size(); i++)
  {
    cv::Vec4i l = unmatched_dynamic[i];
    cv::line(unmatched_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
    unmatched_quantifier += distance_measure(l[0], l[1], l[2], l[3]);
  }
  // find unmatched static lines (red)
  std::vector<cv::Vec4i> unmatched_static = filterEdgesByMatching(dynamic_lines, static_lines);
  for (size_t i = 0; i < unmatched_static.size(); i++)
  {
    cv::Vec4i l = unmatched_static[i];
    cv::line(unmatched_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 2, CV_AA);
    unmatched_quantifier += distance_measure(l[0], l[1], l[2], l[3]);
  }

  int total_alert_level = (int)(unmatched_quantifier);

  cv_bridge::CvImage anomaly_visualization;
  anomaly_visualization.encoding = sensor_msgs::image_encodings::BGR8;
  anomaly_visualization.image = unmatched_cdst;
  ROS_DEBUG("publish visualization");
  visualization_pub->publish(anomaly_visualization.toImageMsg());

  // return the minimized set of alerts
  std::vector<quirkd::Alert> alerts = minimizeAlerts(unmatched_static, unmatched_dynamic, alert);
  return alerts;
}
std::vector<cv::Vec4i> filterEdgesByMatching(std::vector<cv::Vec4i> original, std::vector<cv::Vec4i> new_edges)
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
      double forward_front = distance_measure(seeker[0], seeker[1], candidate[0], candidate[1]);
      double forward_back = distance_measure(seeker[2], seeker[3], candidate[2], candidate[3]);
      if (forward_front < limit / 2 && forward_back < limit / 2 && forward_back + forward_front < limit / 2)
      {
        matched = true;
        break;
      }
      min_best = std::min(min_best, forward_front + forward_back);
      double reverse_front = distance_measure(seeker[0], seeker[1], candidate[2], candidate[3]);
      double reverse_back = distance_measure(seeker[2], seeker[3], candidate[0], candidate[1]);
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
double distance_measure(int start_x, int start_y, int end_x, int end_y)
{
  return pow(start_x - end_x, 2) + pow(start_y - end_y, 2);
}
std::vector<quirkd::Alert> minimizeAlerts(std::vector<cv::Vec4i> unmatched_static,
                                          std::vector<cv::Vec4i> unmatched_dynamic, quirkd::Alert* alert_msg)
{
  std::vector<quirkd::Alert> alerts;
  float map_resolution = 0.05;  // meters per cell/pixel
  for (size_t i = 0; i < unmatched_static.size(); i++)
  {
    cv::Vec4i l = unmatched_static[i];
    quirkd::Alert alert;
    alert.level = (int)(distance_measure(l[0], l[1], l[2], l[3]));
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
    alert.level = (int)(distance_measure(l[0], l[1], l[2], l[3]));
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
std::vector<quirkd::Alert> measureDifference(cv_bridge::CvImage static_image, cv_bridge::CvImage dynamic_image,
                                             quirkd::Alert* alert, image_transport::Publisher* visualization_pub)
{
  quirkd::imagep::preprocessImages(&(static_image.image), &(dynamic_image.image), alert);
  std::vector<quirkd::Alert> result =
      quantifyDifference(&(static_image.image), &(dynamic_image.image), alert, visualization_pub);
  return result;
}

}  // namespace imagep
}  // namespace quirkd