#include <ros/ros.h>

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

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
            *static_image = blurryStatic;
            *dynamic_image = blurryDynamic;
        }
        int quantifyDifference(cv::Mat* static_processed, cv::Mat* dynamic_processed) {
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
            cv::imshow("Absdiff", absdiff);

            // return sum of the absdiff
            return cv::sum(absdiff)[0];
        }
        int measureDifference(cv_bridge::CvImage static_image, cv_bridge::CvImage dynamic_image) {
            // two images (static (base) image and dynamic image)
            // perimeter encoded as part of the alert and images aligned to alert perimeter

            /* Compute difference
             * Steps:
             * - manhattan distance
             */

            // Manhattan Distance
            this->preprocessImages(&(static_image.image), &(dynamic_image.image));
            cv::imshow("Static Map", static_image.image);
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
    image2.image = cv::imread("images/filterInProgress/dynamic001.png", cv::IMREAD_GRAYSCALE);
    int result = it.measureDifference(image1, image2);
    ROS_INFO("ImageTester Exited. Image diff of %d", result);
    return 0;
}
