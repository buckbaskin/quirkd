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

namespace quirkd
{
cv_bridge::CvImagePtr gridToCroppedCvImage(nav_msgs::OccupancyGrid* grid, quirkd::Alert* alert)
{
  // Unpack the Occupancy Grid

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

  // Use CV Bridge to get the CvImagePtr

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(converted_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // TODO cut down, shift images to align with perimeter from alert
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

  int p_cell_origin_x = (int)(alert->min_x / resolution);                // pixels
  int p_cell_origin_y = (int)(alert->min_y / resolution);                // pixels
  int perim_width = (int)((alert->max_x - alert->min_x) / resolution);   // pixels
  int perim_height = (int)((alert->max_y - alert->min_y) / resolution);  // pixels
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
} // namespace quirkd