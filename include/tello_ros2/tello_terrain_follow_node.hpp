#ifndef TELLO_TERRAIN_FOLLOW_HPP_
#define TELLO_TERRAIN_FOLLOW_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TerrainFollow : public rclcpp::Node
{
    public:
        TerrainFollow();
        ~TerrainFollow();
    
    private:
        double _target_alt, _error, _integral;
        double _kp, _ki, _kd;
        double _max_vel, _min_vel;
        double _last_error = 0.0;
        float _offset_z;
        std::string _rel_alt_topic;
        std::string _vert_cmd_vel_topic;
        std::string _offset_topic;
        
        bool _compensate_for_pitch;
        double _pitch;
        std::string _odom_topic;
        
        std::chrono::steady_clock::time_point _last_call_time;
        
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr _rel_alt_sub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _offset_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomerty_sub;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

        void initParameters();
        void initSubscribers();
        void initPublishers();
        void compensatePitch( double &error );

        void relAltitudeCallback( const sensor_msgs::msg::Range &msg );
        void offsetCallback( const std_msgs::msg::Float32 &msg );
        void odometryCallback( const nav_msgs::msg::Odometry &msg );

};

#endif