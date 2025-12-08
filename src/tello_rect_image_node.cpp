
#include <memory>
#include <functional>
#include "tello_ros2/tello_rect_image_node.hpp"

using std::placeholders::_1;

RectImage::RectImage()
  : Node("rect_image")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/tello/image_raw", 10, std::bind(&RectImage::imageCallback, this, _1));
}

void RectImage::imageCallback(const sensor_msgs::msg::Image & msg) const
{
  RCLCPP_INFO(this->get_logger(), "Image Recieved");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectImage>());
  rclcpp::shutdown();
  return 0;
}