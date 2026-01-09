#ifndef TELLO_TERRAIN_FOLLOW_HPP_
#define TELLO_TERRAIN_FOLLOW_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class TerrainFollow : public rclcpp::Node
{
    public:
        TerrainFollow();
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr _rel_alt_sub;

        void relAltitudeCallback(const sensor_msgs::msg::Range &msg) const;
};

#endif