#include "model.cpp"

#include "ros/ros.h"
#include "quirkd/Alert.h"

#include <sstream>

/**
 * Simple sender
 */

int main(int argc, char**argv) {
    ros::init(argc, argv, "map_model_cpp");

    ros::NodeHandle n;
    model::MapHandler* mh = new model::MapHandler();

    ros::Publisher pubber = n.advertise<quirkd::Alert>("/quirkd", 10);

    ros::Rate loop_rate(10);

    int count = 0;
    quirkd::Alert msg;
    while(ros::ok()) {
        msg.num = count;

        std::stringstream ss;
        ss << "hello world " << msg.num;

        ROS_INFO("%s", ss.str().c_str());

        pubber.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}
