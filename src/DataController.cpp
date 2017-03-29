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
            // ros::service::waitForService("static_map");
            static_map_client_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
            // ros::service::waitForService("dynamic_map");
            dynamic_map_client_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");
            image_pub_ = it_.advertise("/quirkd/test/static_image", 1);
        }
        void laserScanCB(const sensor_msgs::LaserScan msg) {
            ROS_INFO("Laser Scan Callback");
            updated = true;
            last_data = msg;
            try {
                tf_.lookupTransform("/map", "/base_laser_link", ros::Time(0), last_tf);
                ROS_INFO("tf success for /map to /base_laser_link");
            } catch (tf::TransformException &ex) {
                ROS_WARN("tf fetch failed. %s", ex.what());
            }
        }
        void update() {
            ROS_INFO("Update Data Processor");
            nav_msgs::GetMap srv;
            cv_bridge::CvImagePtr static_image;
            cv_bridge::CvImagePtr dynamic_image;
            if (static_map_client_.call(srv)) {
                ROS_INFO("Successfull call static map");
                nav_msgs::OccupancyGrid og = srv.response.map;
                static_image = this -> gridToCvImage(&og);

                // For testing

                if (static_image -> image.rows > 60 && static_image -> image.cols>60) {
                    cv::circle(static_image -> image, cv::Point(50,50), 10, CV_RGB(100,100,100), -1);
                }
                ROS_INFO("Publish static image window");
                image_pub_.publish(static_image->toImageMsg());
                
            } else {
                ROS_WARN("Failed to get static map");
            }
            if (dynamic_map_client_.call(srv)) {
                ROS_INFO("Successfull call dynamic map");
                nav_msgs::OccupancyGrid og = srv.response.map;
                dynamic_image = this -> gridToCvImage(&og);
            } else {
                ROS_WARN("Failed to get dynamic map");
            }
            updated = false;
        }
        void pub_alert_status() {
            quirkd::Alert alert;
            alert.level = 0;
            alert.min_x = 0;
            alert.max_x = 1;
            alert.min_y = 0;
            alert.max_y = 1;
            updateAlertPerimeter(&alert, last_data, last_tf);
            alert_pub_.publish(alert);
        }
        bool updated;
        sensor_msgs::LaserScan last_data;
        tf::StampedTransform last_tf;
    private:
        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Publisher image_pub_;
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
                
                live_x = dist * sin(heading);
                live_y = dist * cos(heading);
                alert->min_x = std::min(live_x, double(alert->min_x));
                alert->max_x = std::max(live_x, double(alert->max_x));
                alert->min_y = std::min(live_y, double(alert->min_y));
                alert->max_y = std::max(live_y, double(alert->max_y));

                heading += scan_inc;
            }
        }
        cv_bridge::CvImagePtr gridToCvImage(nav_msgs::OccupancyGrid* grid) {
            // Unpack the Occupancy Grid

            nav_msgs::MapMetaData info = grid -> info;
            float resolution = info.resolution;
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
            std::vector<int8_t> data = grid -> data;
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

            // TODO use CV Bridge to get the CvImagePtr

            cv_bridge::CvImagePtr cv_ptr;

            try {
                cv_ptr = cv_bridge::toCvCopy(converted_image, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            return cv_ptr;
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "DataController");
    ROS_INFO("1");
    DataController dp;
    ROS_INFO("2");
    dp.updated = false;
    ROS_INFO("3");
    ros::Rate r(30);
    ROS_INFO("4");
    dp.update();
    while(ros::ok()) {
        ros::spinOnce();
        if (dp.updated || true) {
            dp.update();
            ROS_INFO("Processed message data in loop");
        }
        dp.pub_alert_status();
        r.sleep();
    }
    ROS_INFO("Data Processor Exited.");
    return 0;
}
