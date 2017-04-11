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

class ImageTester {
    public:
        ImageTester() 
            : it_(n_)
        {
            static_image_pub_ = it_.advertise("/quirkd/test/static_image", 1);
            dynamic_image_pub_ = it_.advertise("/quirkd/test/dynamic_image", 1);
            heatmap_pub_ = it_.advertise("/quirkd/test/heatmap", 1);
        }
        void preprocessImages(cv::Mat* static_image, cv::Mat* dynamic_image) {
            cv::Mat blurryStatic;
            cv::Mat blurryDynamic;
            cv::blur(*static_image, blurryStatic, cv::Size(5, 5));
            cv::blur(*dynamic_image, blurryDynamic, cv::Size(5, 5));
            // *static_image = blurryStatic;
            // *dynamic_image = blurryDynamic;
        }
        int quantifyDifference(cv::Mat* static_processed, cv::Mat* dynamic_processed) {
            // src = static_processed

            ROS_INFO("src size %d x %d", (*static_processed).cols, (*static_processed).rows);
            // cv::imshow("source", *static_processed);
            cv::Mat static_edges, dynamic_edges, static_cdst, dynamic_cdst;
            cv::Canny(*static_processed, static_edges, 50, 200, 3); // TODO check these parameters
            cv::Canny(*dynamic_processed, dynamic_edges, 50, 200, 3);

            cv::imshow("static_edges after Canny", static_edges);
            cv::imshow("dynamic_edges after Canny", dynamic_edges);

            cv::cvtColor(static_edges, static_cdst, CV_GRAY2BGR);
            cv::cvtColor(dynamic_edges, dynamic_cdst, CV_GRAY2BGR);
            
            // Hough Parameters
            std::vector<cv::Vec4i> static_lines, dynamic_lines;
            const int radius_resolution = 1;
            const double theta_resolution = CV_PI/180 / 4;
            const int min_line_length = 8;
            const int threshold = min_line_length; // number of interesections to make a line
            const int max_line_gap = min_line_length - 2; // largest gap (in pixels) considered on the same line
            std::string stitle = "detected lines ";

            cv::HoughLinesP(static_edges, static_lines, radius_resolution, theta_resolution, threshold, min_line_length, max_line_gap);
            for( size_t i = 0; i < static_lines.size(); i++ ) {
                cv::Vec4i l = static_lines[i];
                cv::line( static_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
            }
            cv::HoughLinesP(dynamic_edges, dynamic_lines, radius_resolution, theta_resolution, threshold, min_line_length, max_line_gap);
            for( size_t i = 0; i < dynamic_lines.size(); i++ ) {
                cv::Vec4i l = dynamic_lines[i];
                cv::line( dynamic_cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
            }
            std::ostringstream title;
            title << stitle << min_line_length;
            cv::imshow("static detected", static_cdst);
            cv::imshow("dynamic detected", dynamic_cdst);
            

            // OLD VERSION
            cv::Mat absdiff;
            cv::absdiff(*static_processed, *dynamic_processed, absdiff);
            cv::convertScaleAbs(absdiff, absdiff, 2.55, 0.0);
            
            // use absdiff as a mask to visualize difference with "heatmap" (bgr)
            cv::Mat redBase(static_processed->rows, static_processed->cols, CV_8UC3, cv::Scalar(0, 0, 255));
            cv::Mat byChannel[3];
            cv::split(redBase, byChannel);
            byChannel[2] = absdiff;
            cv::merge(byChannel, 3, redBase);

            cv_bridge::CvImage heatmap;
            heatmap.encoding = sensor_msgs::image_encodings::BGR8;
            heatmap.image = redBase;
            ROS_INFO("publish heatmap");
            heatmap_pub_.publish(heatmap.toImageMsg());
            // cv::imshow("Absdiff", absdiff);

            // return sum of the absdiff
            return cv::sum(absdiff)[0];
        }
        int measureDifference(cv_bridge::CvImage static_image, cv_bridge::CvImage dynamic_image) {
            ROS_INFO("static map original size %d x %d", static_image.image.cols, static_image.image.rows);
            this->preprocessImages(&(static_image.image), &(dynamic_image.image));
            ROS_INFO("static map size %d x %d", static_image.image.cols, static_image.image.rows);
            cv::imshow("Static Map", static_image.image);
            ROS_INFO("dynamic map size %d x %d", dynamic_image.image.cols, dynamic_image.image.rows);
            cv::imshow("Dynamic Map", dynamic_image.image);
            int result = this->quantifyDifference(&(static_image.image), &(dynamic_image.image));
            cv::waitKey(0);
            return result;
        }
    private:
        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Publisher static_image_pub_;
        image_transport::Publisher dynamic_image_pub_;
        image_transport::Publisher heatmap_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ImageTester");
    ImageTester it;
    cv_bridge::CvImage image1;
    cv_bridge::CvImage image2;
    image1.image = cv::imread("images/filterInProgress/static001.png", cv::IMREAD_GRAYSCALE);
    ROS_INFO("imread1 size %d x %d", image1.image.cols, image1.image.rows);
    image2.image = cv::imread("images/filterInProgress/dynamic001.png", cv::IMREAD_GRAYSCALE);
    
    if (!(image1.image.cols > 0 && image1.image.rows > 0 && image2.image.cols > 0 && image2.image.rows > 0)) {
        ROS_ERROR("Did not properly load images");
        return 1;
    }

    int result = it.measureDifference(image1, image2);

    ROS_INFO("ImageTester Exited. Image diff of %d", result);
    return 0;
}
