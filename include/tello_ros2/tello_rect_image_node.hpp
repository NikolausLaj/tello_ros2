#ifndef TELLO_RECT_IMAGES_HPP_
#define TELLO_RECT_IMAGES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/core.hpp"
#include "sensor_msgs/msg/image.hpp"

class RectImage : public rclcpp::Node
{
public:
  RectImage();

private:
  std::vector<double> camera_matrix_values ;
  std::vector<double> distortion_coefficients_values;
  cv::Mat K_;  // camera matrix
  cv::Mat D_;  // distortion coefficients
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  void imageCallback(const sensor_msgs::msg::Image & msg) const;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

#endif  // TELLO_RECT_IMAGES_HPP_