#include "libquirkd/model.h"

#include "ros/ros.h"
#include "quirkd/Alert.h"

#include <sstream>

/**
 * Simple sender
 */

int main(int argc, char**argv) {
    ros::init(argc, argv, "map_model_cpp");

    ros::NodeHandle n;
    MapHandler mh;

    ros::Publisher pubber = n.advertise<quirkd::Alert>("/quirkd", 10);

    ros::Rate loop_rate(10);

    int count = 0;
    quirkd::Alert msg;
    while(ros::ok()) {
        msg.num = count;

        ROS_INFO("%d", msg.num);

        pubber.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}