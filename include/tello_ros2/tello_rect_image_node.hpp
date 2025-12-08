#ifndef TELLO_RECT_IMAGES_HPP_
#define TELLO_RECT_IMAGES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class RectImage : public rclcpp::Node
{
public:
  RectImage();

private:
  void imageCallback(const sensor_msgs::msg::Image & msg) const;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

#endif  // TELLO_RECT_IMAGES_HPP_