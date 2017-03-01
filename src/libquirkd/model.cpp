#include "model.h"

// using costmap_2d::LETHAL_OBSTACLE;

// namespace map_handler {

    SimpleLayer::SimpleLayer() {
        ROS_INFO("Simple Layer Constructor");
        mark_x_ = 0.0;
        mark_y_ = 0.0;
    }
    SimpleLayer::onInitialize() {
        ROS_INFO("on init");
    }
    SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double min_x, double min_y, double max_x, double max_y) {
        ROS_INFO("update bounds");
    }
    SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
        ROS_INFO("update costs");
    }
    SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level) {
        ROS_INFO("reconfigureCB");
    }

    MapHandler::MapHandler() {
        ROS_INFO("MapHandler constructor");
        SimpleLayer scan_layer = new SimpleLayer();
        bar = 10;
    }
    MapHandler foo() {
        bar += 1;
    }

// } // end namespace map_handler

MapHandler::MapHandler() {
    ROS_INFO("MapHandler constructor 2");
}
