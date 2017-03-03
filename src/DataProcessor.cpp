#include <ros/ros.h>
#include <quirkd/Alert.h>
#include <sensor_msgs/LaserScan.h>

class DataProcessor {
    public:
        DataProcessor() {
            alert_pub_ = n_.advertise<quirkd::Alert>("/quirkd/alert/notification", 1);
            laser_sub_ = n_.subscribe("/base_scan", 1, &DataProcessor::laserScanCB, this);
        }
        void laserScanCB(const sensor_msgs::LaserScan msg) {
            ROS_INFO("Laser Scan Callback");
            updated = true;
            last_data = msg;
        }
        void update() {
           ROS_INFO("Update Data Processor");
           updated = false;
        }
        bool updated;
        sensor_msgs::LaserScan last_data;
    private:
        ros::NodeHandle n_;
        ros::Publisher alert_pub_;
        ros::Subscriber laser_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "DataProcessor");
    ROS_INFO("1");
    DataProcessor dp;
    ROS_INFO("2");
    dp.updated = false;
    ROS_INFO("3");
    ros::Rate r(30);
    ROS_INFO("4");
    while(ros::ok()) {
        ros::spinOnce();
        if (dp.updated) {
            dp.update();
            ROS_INFO("Processed message data in loop");
        }
        r.sleep();
    }
    ROS_INFO("Data Processor Exited.");
    return 0;
}
