#include "tello_ros2/tello_terrain_follow_node.hpp"

using std::placeholders::_1;

TerrainFollow::TerrainFollow() : Node("tello_terrain_follow_node")
{
    // Read Parameters
    this->declare_parameter("target_distance", 1.0);
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("ki", 0.1);
    this->declare_parameter("kd", 0.05);
    this->declare_parameter("min_vel", -1.0);
    this->declare_parameter("max_vel", 1.0);
    this->declare_parameter("rel_alt_topic", "");
    this->declare_parameter("cmd_vel", "");

    _target_alt = this->get_parameter("target_distance").as_double();
    _kp = this->get_parameter("kp").as_double();
    _ki = this->get_parameter("ki").as_double();
    _kd = this->get_parameter("kd").as_double();
    _min_vel = this->get_parameter("min_vel").as_double();
    _max_vel = this->get_parameter("max_vel").as_double();
    _rel_alt_topic = this->get_parameter("rel_alt_topic").as_string();
    _vert_cmd_vel_topic = this->get_parameter("cmd_vel").as_string();
   

    // Declare Subscribers
    _rel_alt_sub = this->create_subscription<sensor_msgs::msg::Range>(
        _rel_alt_topic, 10, std::bind(&TerrainFollow::relAltitudeCallback, this, _1));
        
    // Declare Publishers
    _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        _vert_cmd_vel_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), "Terrain Follow Node Initalized and Running!");
    RCLCPP_INFO(this->get_logger(), "Parameters\n: Tagert Distance: %f\n: Kp: %f\n: Ki: %f\n: Kd: %f\n: Min Vel: %f\n: Max Vel: %f\n: Rel Alt Topic: %s\n: Cmd Vel Topic: %s",
        _target_alt, _kp, _ki, _kd, _min_vel, _max_vel, _rel_alt_topic.c_str(), _vert_cmd_vel_topic.c_str());
}


void TerrainFollow::relAltitudeCallback(const sensor_msgs::msg::Range & msg) const
{
    // implement PI/PID Controller here to maintain a certain alitude above ground
    std::cout << "Current Distance: "<< msg.range << std::endl;
    geometry_msgs::msg::Twist cmd_vel_msg;
    double error = _target_alt - msg.range;
    double control_signal = _kp * error; // + _ki * integral + _kd * derivative;

    if (control_signal > _max_vel)
    {
        control_signal = _max_vel;
    }
    else if (control_signal < _min_vel)
    {
        control_signal = _min_vel;
    }

    cmd_vel_msg.linear.z = control_signal;
    _cmd_vel_pub->publish(cmd_vel_msg);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainFollow>());
  rclcpp::shutdown();
  return 0;
}