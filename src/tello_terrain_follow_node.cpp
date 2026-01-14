#include "tello_ros2/tello_terrain_follow_node.hpp"

using std::placeholders::_1;

TerrainFollow::TerrainFollow() : Node("tello_terrain_follow_node")
{
    TerrainFollow::initParameters();
    TerrainFollow::initSubscribers();
    TerrainFollow::initPublishers();
    
    RCLCPP_INFO(this->get_logger(), "Terrain Follow Node Initalized and Running!");
    RCLCPP_INFO(this->get_logger(), "Parameters\n: Tagert Distance: %f\n: Kp: %f\n: Ki: %f\n: Kd: %f\n: Min Vel: %f\n: Max Vel: %f\n: Rel Alt Topic: %s\n: Cmd Vel Topic: %s",
        _target_alt, _kp, _ki, _kd, _min_vel, _max_vel, _rel_alt_topic.c_str(), _vert_cmd_vel_topic.c_str());

    _last_call_time = std::chrono::steady_clock::now();
}


TerrainFollow::~TerrainFollow()
{
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.z = 0.0;
    _cmd_vel_pub->publish(stop_msg);
    rclcpp::spin_some(shared_from_this());
}


void TerrainFollow::initParameters()
{
    this->declare_parameter("target_distance", 1.0);
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("ki", 0.1);
    this->declare_parameter("kd", 0.05);
    this->declare_parameter("min_vel", -1.0);
    this->declare_parameter("max_vel", 1.0);
    this->declare_parameter("rel_alt_topic", "");
    this->declare_parameter("cmd_vel", "");
    this->declare_parameter("offset_topic", "");
    this->declare_parameter("compensate_for_pitch", false);
    this->declare_parameter("odom_topic", "");

    _target_alt = this->get_parameter("target_distance").as_double();
    _kp = this->get_parameter("kp").as_double();
    _ki = this->get_parameter("ki").as_double();
    _kd = this->get_parameter("kd").as_double();
    _min_vel = this->get_parameter("min_vel").as_double();
    _max_vel = this->get_parameter("max_vel").as_double();
    _rel_alt_topic = this->get_parameter("rel_alt_topic").as_string();
    _offset_topic = this->get_parameter("offset_topic").as_string();
    _vert_cmd_vel_topic = this->get_parameter("cmd_vel").as_string();
    _compensate_for_pitch = this->get_parameter("compensate_for_pitch").as_bool();
    _odom_topic = this->get_parameter("odom_topic").as_string();
}


void TerrainFollow::initSubscribers()
{
    _rel_alt_sub = this->create_subscription<sensor_msgs::msg::Range>(
        _rel_alt_topic, 10, std::bind(&TerrainFollow::relAltitudeCallback, this, _1));
    
    _offset_sub = this->create_subscription<std_msgs::msg::Float32>(
        _offset_topic, 10, std::bind(&TerrainFollow::offsetCallback, this, _1));

    _odomerty_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        _odom_topic, 10, std::bind(&TerrainFollow::odometryCallback, this, _1));
}


void TerrainFollow::initPublishers()
{
    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    _vert_cmd_vel_topic, 10);
}


void TerrainFollow::relAltitudeCallback(const sensor_msgs::msg::Range & msg)
{
    // todo angle compensation
    geometry_msgs::msg::Twist cmd_vel_msg;
    
    auto current_call_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>( current_call_time - _last_call_time ).count();
    
    if (_compensate_for_pitch == false)
    {
        _error = _target_alt - (msg.range - _offset_z);
    }
    _integral += _error * dt;
    double derivative = ( _error - _last_error ) / dt;
    double control_signal = _kp * _error + _ki * _integral + _kd * derivative;

    if ( control_signal > _max_vel )
    {
        control_signal = _max_vel;
    }
    else if ( control_signal < _min_vel )
    {
        control_signal = _min_vel;
    }

    cmd_vel_msg.linear.z = control_signal;
    _cmd_vel_pub->publish(cmd_vel_msg);
    _last_call_time = current_call_time;
    _last_error = _error;
}


void TerrainFollow::offsetCallback( const std_msgs::msg::Float32 &msg )
{
    _offset_z = msg.data;
    RCLCPP_INFO(this->get_logger(), "Offset set to %fm", _offset_z);
}


void TerrainFollow::odometryCallback( const nav_msgs::msg::Odometry &msg )
{
    _pitch = msg.pose.pose.orientation.w;
}


void TerrainFollow::compensatePitch(double &error)
{

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainFollow>());
  rclcpp::shutdown();
  return 0;
}
