#include <ros/ros.h>

#include <math.h>
#include <nav_msgs/GetMap.h>
#include <quirkd/Alert.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class DataController {
    public:
        DataController() {
            alert_pub_ = n_.advertise<quirkd::Alert>("/quirkd/alert/notification", 1);
            laser_sub_ = n_.subscribe("/base_scan", 1, &DataController::laserScanCB, this);
            // ros::service::waitForService("static_map");
            static_map_client_ = n_.serviceClient<nav_msgs::GetMap>("static_map");
            // ros::service::waitForService("dynamic_map");
            dynamic_map_client_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");
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
            if (static_map_client_.call(srv)) {
                ROS_INFO("Successfull call static map");
            } else {
                ROS_WARN("Failed to get static map");
            }
            if (dynamic_map_client_.call(srv)) {
                ROS_INFO("Successfull call dynamic map");
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
        if (dp.updated) {
            dp.update();
            ROS_INFO("Processed message data in loop");
        }
        dp.pub_alert_status();
        r.sleep();
    }
    ROS_INFO("Data Processor Exited.");
    return 0;
}
