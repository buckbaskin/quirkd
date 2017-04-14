#include <ros/ros.h>

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/highgui/highgui.hpp>
#include <quirkd/Alert.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class ImageTester
{
public:
  ImageTester() : it_(n_)
  {
    static_image_pub_ = it_.advertise("/quirkd/test/static_image", 1);
    dynamic_image_pub_ = it_.advertise("/quirkd/test/dynamic_image", 1);
    visualization_pub_ = it_.advertise("/quirkd/test/visualization_image", 1);
  }
  void preprocessImages(cv::Mat* static_image, cv::Mat* dynamic_image)
  {
  }
  int quantifyDifference(cv::Mat* static_processed, cv::Mat* dynamic_processed)
  {
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
    cv::imshow("static detected", static_cdst);
    cv::imshow("dynamic detected", dynamic_cdst);

    double unmatched_quantifier = 0.0;

    // find unmatched dynamic lines (red)
    std::vector<cv::Vec4i> unmatched_lines = this->filterEdgesByMatching(static_lines, dynamic_lines);
    for (size_t i = 0; i < unmatched_lines.size(); i++)
    {
      cv::Vec4i l = unmatched_lines[i];
      cv::line(unmatched_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
      unmatched_quantifier += this->distance_measure(l[0], l[1], l[2], l[3]);
    }
    // find unmatched static lines (red)
    unmatched_lines = this->filterEdgesByMatching(dynamic_lines, static_lines);
    for (size_t i = 0; i < unmatched_lines.size(); i++)
    {
      cv::Vec4i l = unmatched_lines[i];
      cv::line(unmatched_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 2, CV_AA);
      unmatched_quantifier += this->distance_measure(l[0], l[1], l[2], l[3]);
    }
    cv::imshow("unmatched", unmatched_cdst);

    int alert_level = (int)(unmatched_quantifier);

    cv_bridge::CvImage anomaly_visualization;
    anomaly_visualization.encoding = sensor_msgs::image_encodings::BGR8;
    anomaly_visualization.image = unmatched_cdst;
    ROS_INFO("publish visualization");
    visualization_pub_.publish(anomaly_visualization.toImageMsg());

    // return sum of the absdiff
    return alert_level;
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
        ROS_INFO("Best min %f", min_best);
        unmatched_set.push_back(seeker);
      }
      else
      {
        // I found a good match!
      }
    }
    return unmatched_set;
  }
  int measureDifference(cv_bridge::CvImage static_image, cv_bridge::CvImage dynamic_image)
  {
    this->preprocessImages(&(static_image.image), &(dynamic_image.image));
    int result = this->quantifyDifference(&(static_image.image), &(dynamic_image.image));
    cv::waitKey(0);
    return result;
  }

private:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Publisher static_image_pub_;
  image_transport::Publisher dynamic_image_pub_;
  image_transport::Publisher visualization_pub_;
  double distance_measure(int start_x, int start_y, int end_x, int end_y)
  {
    return pow(start_x - end_x, 2) + pow(start_y - end_y, 2);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ImageTester");
  ImageTester it;
  cv_bridge::CvImage image1;
  cv_bridge::CvImage image2;
  image1.image = cv::imread("images/filterInProgress/static001.png", cv::IMREAD_GRAYSCALE);
  ROS_INFO("imread1 size %d x %d", image1.image.cols, image1.image.rows);
  image2.image = cv::imread("images/filterInProgress/dynamic001.png", cv::IMREAD_GRAYSCALE);

  if (!(image1.image.cols > 0 && image1.image.rows > 0 && image2.image.cols > 0 && image2.image.rows > 0))
  {
    ROS_ERROR("Did not properly load images");
    return 1;
  }

  int result = it.measureDifference(image1, image2);

  ROS_INFO("ImageTester Exited. Image diff of %d", result);
  for (int i = 0; i < 10000; ++i)
  {
  }
  return 0;
}
