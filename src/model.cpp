#include <quirkd/model.h>

// using costmap_2d::LETHAL_OBSTACLE;

// namespace model {

    SimpleLayer::SimpleLayer() {
        ROS_INFO("Simple Layer Constructor");
        this -> mark_x_ = 0.0;
        this -> mark_y_ = 0.0;
    }
    void SimpleLayer::onInitialize() {
        ROS_INFO("on init");
    }
    void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double min_x, double min_y, double max_x, double max_y) {
        ROS_INFO("update bounds");
    }
    void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
        ROS_INFO("update costs");
    }
    void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level) {
        ROS_INFO("reconfigureCB");
    }

    MapHandler::MapHandler() {
        ROS_INFO("MapHandler constructor");
        SimpleLayer* scan_layer = new SimpleLayer();
        this->bar = 10;
    }
    void MapHandler::foo() {
        this->bar += 1;
    }

// } // end namespace model

