#ifndef TELLO_TERRAIN_FOLLOW_HPP_
#define TELLO_TERRAIN_FOLLOW_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TerrainFollow : public rclcpp::Node
{
    public:
        TerrainFollow();
    
    private:
        double _target_alt;
        double _kp, _ki, _kd;
        double _max_vel, _min_vel;
        std::string _rel_alt_topic;
        std::string _vert_cmd_vel_topic;

        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr _rel_alt_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

        void relAltitudeCallback(const sensor_msgs::msg::Range &msg) const;
};

#endif