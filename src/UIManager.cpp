#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "UIManager");
    ROS_INFO("Init in UIManager");
    ros::NodeHandle n;
    ros::Publisher polygonPub = n.advertise<geometry_msgs::PolygonStamped>("AlertViz", 1);

    geometry_msgs::PolygonStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "map";

    geometry_msgs::Point32 p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    ps.polygon.points.push_back(p1);
    geometry_msgs::Point32 p2;
    p2.x = 0;
    p2.y = 10;
    p2.z = 0;
    ps.polygon.points.push_back(p2);
    geometry_msgs::Point32 p3;
    p3.x = 10;
    p3.y = 10;
    p3.z = 0;
    ps.polygon.points.push_back(p3);
    geometry_msgs::Point32 p4;
    p4.x = 10;
    p4.y = 0;
    p4.z = 0;
    ps.polygon.points.push_back(p4);

    ros::Rate r(10);
    while (ros::ok()) {    
        polygonPub.publish(ps);
        ROS_INFO("Publish ps");
        r.sleep();
        ROS_INFO("Rate sleep end");
    }
}
