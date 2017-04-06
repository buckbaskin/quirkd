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

class DataController {
    public:
        DataController() 
            : it_(n_)
        {
            alert_pub_ = n_.advertise<quirkd::Alert>("/quirkd/alert/notification", 1);
            laser_sub_ = n_.subscribe("/base_scan", 1, &DataController::laserScanCB, this);
            ROS_INFO("WaitForService(\"static_map\");");
            ros::service::waitForService("static_map");
            static_map_client_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
            ROS_INFO("WaitForService(\"dynamic_map\");");
            ros::service::waitForService("dynamic_map");
            dynamic_map_client_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");
            static_image_pub_ = it_.advertise("/quirkd/test/static_image", 1);
            dynamic_image_pub_ = it_.advertise("/quirkd/test/dynamic_image", 1);
            heatmap_pub_ = it_.advertise("/quirkd/test/heatmap", 1);
        }
        void laserScanCB(const sensor_msgs::LaserScan msg) {
            ROS_INFO("Laser Scan Callback");
            updated = true;
            last_data = msg;
            try {
                ROS_INFO("Waiting for transform");
                tf_.waitForTransform("/map", "/base_laser_link", ros::Time(0), ros::Duration(3.0));
                tf_.lookupTransform("/map", "/base_laser_link", ros::Time(0), last_tf);
                ROS_INFO("tf success for /map to /base_laser_link");
            } catch (tf::TransformException &ex) {
                ROS_WARN("tf fetch failed. %s", ex.what());
            }
        }
        void update() {
            quirkd::Alert alert;
            alert.level = 0;
            alert.min_x = 0;
            alert.max_x = 1;
            alert.min_y = 0;
            alert.max_y = 1;
            updateAlertPerimeter(&alert, last_data, last_tf);

            ROS_INFO("Update Data Processor");
            nav_msgs::GetMap srv;
            cv_bridge::CvImagePtr static_image;
            cv_bridge::CvImagePtr dynamic_image;
            if (static_map_client_.call(srv)) {
                ROS_INFO("Successfull call static map");
                nav_msgs::OccupancyGrid og = srv.response.map;
                static_image = this->gridToCroppedCvImage(&og, &alert);

                static_image_pub_.publish(static_image->toImageMsg());
                
            } else {
                ROS_WARN("Failed to get static map");
            }
            if (dynamic_map_client_.call(srv)) {
                ROS_INFO("Successfull call dynamic map");
                nav_msgs::OccupancyGrid og = srv.response.map;
                dynamic_image = this->gridToCroppedCvImage(&og, &alert);

                dynamic_image_pub_.publish(dynamic_image->toImageMsg());
            } else {
                ROS_WARN("Failed to get dynamic map");
                dynamic_image = static_image;
            }

            // two images (static (base) image and dynamic image)
            // perimeter encoded as part of the alert and images aligned to alert perimeter

            /* Compute difference
             * Steps:
             * - manhattan distance
             */

            // Manhattan Distance
            cv::Mat blurryStatic;
            cv::Mat blurryDynamic;
            cv::blur(static_image->image, blurryStatic, cv::Size(5, 5));
            cv::blur(dynamic_image->image, blurryDynamic, cv::Size(5, 5));
            cv::Mat absdiff;
            cv::absdiff(blurryStatic, blurryDynamic, absdiff);
            cv::convertScaleAbs(absdiff, absdiff, 2.55, 0.0);
            ROS_INFO("Scale by 2.55");

            // use absdiff as a mask to visualize difference with "heatmap" (bgr)
            cv::Mat redBase(static_image->image.rows, static_image->image.cols, CV_8UC3, cv::Scalar(0, 0, 255));
            cv::Mat byChannel[3];
            cv::split(redBase, byChannel);
            byChannel[2] = absdiff;
            cv::merge(byChannel, 3, redBase);

            cv_bridge::CvImage heatmap;
            heatmap.header = static_image->header;
            heatmap.encoding = sensor_msgs::image_encodings::BGR8;
            heatmap.image = redBase;
            ROS_INFO("publish heatmap part 2");
            heatmap_pub_.publish(heatmap.toImageMsg());
            
            alert.level = cv::sum(absdiff)[0];

            // TODO publish difference
            ROS_INFO("The two images have a difference of %d", alert.level);
            alert_pub_.publish(alert);

            updated = false;
        }
        bool updated;
        sensor_msgs::LaserScan last_data;
        tf::StampedTransform last_tf;
    private:
        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Publisher static_image_pub_;
        image_transport::Publisher dynamic_image_pub_;
        image_transport::Publisher heatmap_pub_;
        ros::Publisher alert_pub_;
        ros::Subscriber laser_sub_;
        ros::ServiceClient dynamic_map_client_;
        ros::ServiceClient static_map_client_;
        tf::TransformListener tf_;
        void updateAlertPerimeter(quirkd::Alert* alert, const sensor_msgs::LaserScan scan, const tf::StampedTransform tf) {
            double base_x = tf.getOrigin().x();
            double base_y = tf.getOrigin().y();
            double r, p, base_heading;
            tf::Matrix3x3 m(tf.getRotation());
            m.getRPY(r, p, base_heading);

            double scan_min = base_heading + scan.angle_min;
            double scan_inc = scan.angle_increment;
            double heading = scan_min;

            alert->min_x = base_x;
            alert->max_x = base_x;
            alert->min_y = base_y;
            alert->max_y = base_y;

            double live_x, live_y;

            for (int i = 0; i < scan.ranges.size(); i++) {
                double dist = scan.ranges[i];
                
                live_x = base_x + dist * cos(heading);
                live_y = base_y + dist * sin(heading);
                alert->min_x = std::min(live_x, double(alert->min_x));
                alert->max_x = std::max(live_x, double(alert->max_x));
                alert->min_y = std::min(live_y, double(alert->min_y));
                alert->max_y = std::max(live_y, double(alert->max_y));

                heading += scan_inc;
            }
            double padding = 0.5;
            alert->min_x += -padding;
            alert->min_y += -padding;
            alert->max_x += padding;
            alert->max_y += padding;
        }
        cv_bridge::CvImagePtr gridToCroppedCvImage(nav_msgs::OccupancyGrid* grid, quirkd::Alert* alert) {
            // Unpack the Occupancy Grid

            nav_msgs::MapMetaData info = grid->info;
            float resolution = info.resolution; // meters per pixel
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
            unsigned_data.resize(data.size()); // allocate and fill vector to match num elements of data

            for (std::vector<int8_t>::size_type i = 0; i < data.size(); i++) {
                int8_t old_int = data[i];
                // assume unknown values (signed -1) are just 0
                // assumption is that unseen parts of the map have no obstacles until proven otherwise
                uint8_t new_int = (uint8_t)((old_int == -1) ? 0 : old_int);
                unsigned_data[(std::vector<uint8_t>::size_type) i] = new_int;
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

            try {
                cv_ptr = cv_bridge::toCvCopy(converted_image, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception& e) {
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
            int map_cell_origin_x = (int) (origin_x / resolution); // pixels
            int map_cell_origin_y = (int) (origin_y / resolution); // pixels
            
            cv::Rect map_region(map_cell_origin_x, map_cell_origin_y, cell_width, cell_height);
            ROS_DEBUG("map x %d y %d w %d h %d", map_region.x, map_region.y, map_region.width, map_region.height);

            int p_cell_origin_x = (int) (alert->min_x / resolution); // pixels
            int p_cell_origin_y = (int) (alert->min_y / resolution); // pixels
            int perim_width = (int) ((alert->max_x - alert->min_x) / resolution); // pixels
            int perim_height = (int) ((alert->max_y - alert->min_y) / resolution); // pixels
            ROS_DEBUG("perim w: %d h: %d", perim_width, perim_height);

            cv::Rect perimeter(p_cell_origin_x, p_cell_origin_y, perim_width, perim_height);
            ROS_DEBUG("perimeter x %d y %d w %d h %d", perimeter.x, perimeter.y, perimeter.width, perimeter.height);

            cv::Rect intersect = map_region & perimeter;

            ROS_DEBUG("I made my map rectangle");

            // shift intersection to the coordinates of the cropped image
            intersect.x -= map_cell_origin_x;
            intersect.y -= map_cell_origin_y;

            ROS_DEBUG("shift intersect 1 x %d y %d w %d h %d", intersect.x, intersect.y, intersect.width, intersect.height);

            // Assertion: 0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= m.rows
            ROS_DEBUG("conditions 1: %d %d %d %d %d %d",
                0 <= intersect.x,
                0 <= intersect.width,
                intersect.x + intersect.width <= cv_ptr->image.cols,
                0 <= intersect.y,
                0 <= intersect.height,
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

            // Assertion: 0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= m.rows
            ROS_DEBUG("conditions 2: %d %d %d %d %d %d",
                0 <= intersect.x,
                0 <= intersect.width,
                intersect.x + intersect.width <= base.cols,
                0 <= intersect.y,
                0 <= intersect.height,
                intersect.y + intersect.height <= base.rows);
            ROS_DEBUG("conditions 2.1:\n0 <= %d\n0 <= %d\n%d <= %d\n0 <= %d\n0 <= %d\n%d <= %d",
                intersect.x,
                intersect.width,
                intersect.x + intersect.width, base.cols,
                0 <= intersect.y,
                0 <= intersect.height,
                intersect.y + intersect.height, base.rows);
            
            cv::Mat base_roi(base, intersect);

            ROS_DEBUG("shift intersect 2");

            // copy the map overlay to the base image
            if (intersect.area() > 0) {
                cropped_map.copyTo(base_roi);
                ROS_DEBUG("Overlay done");
            } else {
                ROS_DEBUG("No overlay (no area)");
            }

            cv_ptr->image = base;
            return cv_ptr;
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "DataController");
    DataController dp;
    dp.updated = false;
    ros::Rate r(30);

    dp.update();
    while(ros::ok()) {
        ros::spinOnce();
        if (dp.updated || true) {
            dp.update();
            ROS_INFO("Processed message data in loop");
        }
        // dp.pub_alert_status();
        r.sleep();
    }
    ROS_INFO("Data Processor Exited.");
    return 0;
}
