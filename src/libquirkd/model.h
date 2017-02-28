#ifndef __MAPHANDLER_H_INCLUDED__
#define __MAPHANDLER_H_INCLUDED__
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace map_handler {

    class SimpleLayer {
        public:
            SimpleLayer();
            virtual void onInitialize();
            virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double min_x, double min_y, double max_x, double max_y);
            virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
        private:
            void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
            double mark_x_, mark_y;
            dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    };

    class MapHandler {
        public:
            void foo();
            int bar;
    };

}

#endif // __MAPHANDLER_H_INCLUDED__
