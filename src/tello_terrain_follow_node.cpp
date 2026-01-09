#include "tello_ros2/tello_terrain_follow_node.hpp"

using std::placeholders::_1;

TerrainFollow::TerrainFollow() : Node("tello_terrain_follow_node")
{
    _rel_alt_sub = this->create_subscription<sensor_msgs::msg::Range>(
        "tello/relative_altitude", 10, std::bind(&TerrainFollow::relAltitudeCallback, this, _1));
}

void TerrainFollow::relAltitudeCallback(const sensor_msgs::msg::Range & msg) const
{
    std::cout << "Current Distance: "<< msg.range << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainFollow>());
  rclcpp::shutdown();
  return 0;
}