#include <ros/ros.h>

#include <nav_msgs/GetMap.h>
#include <quirkd/Alert.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

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
            quirkd::Alert msg;
            alert_pub_.publish(msg);
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
