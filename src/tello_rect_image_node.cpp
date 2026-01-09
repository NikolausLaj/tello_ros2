
#include <memory>
#include <functional>

#include "tello_ros2/tello_rect_image_node.hpp"

using std::placeholders::_1;

RectImage::RectImage()
  : Node("tello_rect_image_node")
{
 // Declare parameters (required in ROS2)
  this->declare_parameter("camera_matrix.rows", 0);
  this->declare_parameter("camera_matrix.cols", 0);
  this->declare_parameter("camera_matrix.data", std::vector<double>());

  this->declare_parameter("distortion_coefficients.rows", 0);
  this->declare_parameter("distortion_coefficients.cols", 0);
  this->declare_parameter("distortion_coefficients.data", std::vector<double>());

  // Read parameters
  int k_rows = this->get_parameter("camera_matrix.rows").as_int();
  int k_cols = this->get_parameter("camera_matrix.cols").as_int();
  auto k_data = this->get_parameter("camera_matrix.data").as_double_array();

  int d_rows = this->get_parameter("distortion_coefficients.rows").as_int();
  int d_cols = this->get_parameter("distortion_coefficients.cols").as_int();
  auto d_data = this->get_parameter("distortion_coefficients.data").as_double_array();

  // Convert to OpenCV matrices
  K_ = cv::Mat(k_rows, k_cols, CV_64F, k_data.data()).clone();
  D_ = cv::Mat(d_rows, d_cols, CV_64F, d_data.data()).clone();

  RCLCPP_INFO(this->get_logger(), "Camera Matrix K:\n%f %f %f\n%f %f %f\n%f %f %f",
      K_.at<double>(0,0), K_.at<double>(0,1), K_.at<double>(0,2),
      K_.at<double>(1,0), K_.at<double>(1,1), K_.at<double>(1,2),
      K_.at<double>(2,0), K_.at<double>(2,1), K_.at<double>(2,2)  
  );

  RCLCPP_INFO(this->get_logger(), "Distortion D: [%f, %f, %f, %f, %f]",
      D_.at<double>(0,0), D_.at<double>(0,1), D_.at<double>(0,2),
      D_.at<double>(0,3), D_.at<double>(0,4)
  );

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/tello/image_raw", 10, std::bind(&RectImage::imageCallback, this, _1));

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("tello/image_undistorted", 10);

}

void RectImage::imageCallback(const sensor_msgs::msg::Image & msg) const
{
  RCLCPP_INFO(this->get_logger(), "Undistort Image Here!!! :)");
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
  cv::Mat undistorted_image;
  cv::undistort(cv_ptr->image, undistorted_image, K_, D_);
  cv_ptr->image = undistorted_image;
  image_pub_->publish(*cv_ptr->toImageMsg());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectImage>());
  rclcpp::shutdown();
  return 0;
}