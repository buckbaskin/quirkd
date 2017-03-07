#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <quirkd/Alert.h>

class UIManager {
    public:
        UIManager() {
            ros::NodeHandle n;
            lowPub = n.advertise<geometry_msgs::PolygonStamped>("/lowAlert", 1);
            warnPub = n.advertise<geometry_msgs::PolygonStamped>("/warnAlert", 1);
            maxPub = n.advertise<geometry_msgs::PolygonStamped>("/maxAlert", 1);
            alertSub = n.subscribe("/quirkd/alert/notification", 1, &UIManager::alertCB, this);
            ROS_INFO("Done with UIManager constructor");
        }
        void alertCB(const quirkd::Alert alert) {
            ROS_INFO("Alert CB");
            if (alert.level < 10) {
                publishPolygon(&lowPub, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
            } else if (alert.level > 100) {
                publishPolygon(&maxPub, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
            } else {
                publishPolygon(&warnPub, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
            }
        }
        void publishPolygon(ros::Publisher* pub, float min_x, float max_x, float min_y, float max_y) {
            ROS_INFO("Publish polygon");
            geometry_msgs::PolygonStamped ps;
            ps.header.stamp = ros::Time::now();
            ps.header.frame_id = "map";

            geometry_msgs::Point32 p1;
            p1.x = min_x;
            p1.y = min_y;
            p1.z = 0;
            ps.polygon.points.push_back(p1);
            geometry_msgs::Point32 p2;
            p2.x = max_x;
            p2.y = min_y;
            p2.z = 0;
            ps.polygon.points.push_back(p2);
            geometry_msgs::Point32 p3;
            p3.x = max_x;
            p3.y = max_y;
            p3.z = 0;
            ps.polygon.points.push_back(p3);
            geometry_msgs::Point32 p4;
            p4.x = min_x;
            p4.y = max_y;
            p4.z = 0;
            ps.polygon.points.push_back(p4);

            pub->publish(ps);
        }
    private:
        ros::NodeHandle n;
        ros::Publisher lowPub;
        ros::Publisher warnPub;
        ros::Publisher maxPub;
        ros::Subscriber alertSub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "UIManager");
    ROS_INFO("Init in UIManager");
    UIManager ui;
    ros::spin();
}
