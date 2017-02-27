#include "ros/ros.h"
#include "std_msgs/String.h"
#include "quirkd/Alert.h"

#include <sstream>

/**
 * Simple sender
 */

int main(int argc, char**argv) {
    ros::init(argc, argv, "MapModel_cpp");

    ros::NodeHandle n;

    ros::Publisher pubber = n.advertise<std_msgs::String>("/quirkd", 10);

    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        pubber.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
