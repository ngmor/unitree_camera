// This publishes camerainfo for one left/right stereo pair. 

#include <memory>
#include <string>
#include <vector>
#include <exception>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "UnitreeCameraSDK.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;


class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher()
  : Node("info_publisher")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The frame rate of the camera (fps).";
    declare_parameter("fps", 30, param);
    fps_ = get_parameter("fps").get_parameter_value().get<int>();
    interval_ = 1.0 / static_cast<double>(fps_);
    interval_ms_ = static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0));

    param.description = "Left camera frame width";
    declare_parameter("left_image_width", 0, param);
    left_image_width = get_parameter("left_image_width").get_parameter_value().get<int>();

    param.description = "Left camera frame height";
    declare_parameter("left_image_height", 0, param);
    left_image_height = get_parameter("left_image_height").get_parameter_value().get<int>();

    param.description = "Left Camera name";
    declare_parameter("left_camera_name", "", param);
    left_camera_name = get_parameter("left_camera_name").get_parameter_value().get<std::string>();

    param.description = "Left Camera matrix";
    declare_parameter("left_camera_matrix", std::vector<double>{}, param);
    left_camera_matrix = get_parameter("left_camera_matrix").get_parameter_value().get<std::vector<double>>();

    param.description = "Left Camera distortion model";
    declare_parameter("left_distortion_model", "", param);
    left_distortion_model = get_parameter("left_distortion_model").get_parameter_value().get<std::string>();

    param.description = "Left Camera distortion coefficients";
    declare_parameter("left_distortion_coefficients", std::vector<double>{}, param);
    left_distortion_coefficients = get_parameter("left_distortion_coefficients").get_parameter_value().get<std::vector<double>>();

    param.description = "Left Camera rectification matrix";
    declare_parameter("left_rectification_matrix", std::vector<double>{}, param);
    left_rectification_matrix = get_parameter("left_rectification_matrix").get_parameter_value().get<std::vector<double>>();

    param.description = "Left Camera projection matrix matrix";
    declare_parameter("left_projection_matrix", std::vector<double>{}, param);
    left_projection_matrix = get_parameter("left_projection_matrix").get_parameter_value().get<std::vector<double>>();


    // Publisher
    pub_info_left_ = create_publisher<sensor_msgs::msg::CameraInfo>("info_left", 10);
    pub_info_right_ = create_publisher<sensor_msgs::msg::CameraInfo>("info_right", 10);
    // Timer
    timer_ = create_wall_timer(interval_ms_, std::bind(&InfoPublisher::timer_callback, this));

  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_left_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_right_;

  double interval_;
  std::chrono::milliseconds interval_ms_;
  int fps_;

  // stuff I added
  int left_image_width, left_image_height;
  std::string left_camera_name, left_distortion_model;
  std::vector<double> left_camera_matrix, left_distortion_coefficients, 
                      left_rectification_matrix, left_projection_matrix;

  int right_image_width, right_image_height;
  std::string right_camera_name, right_distortion_model;
  std::vector<double> right_camera_matrix, right_distortion_coefficients, 
                      right_rectification_matrix, right_projection_matrix;

  sensor_msgs::msg::CameraInfo camera_info_left_;
  sensor_msgs::msg::CameraInfo camera_info_right_;

  void timer_callback()
  {

    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    camera_info_left_.header = header;
    camera_info_right_.header = header;

    // things to set:
    // camera_info_.height
    // camera_info_.width
    // camera_info_.distortion_model
    // camera_info_.d
    // camera_info_.k
    // camera_info_.r
    // camera_info_.p

    // camera_info_left_.height = left_image_height;
    // camera_info_left_.width = left_image_width;
    // camera_info_left_.distortion_model = left_distortion_model;
    // camera_info_left_.d = left_distortion_coefficients;
    // camera_info_left_.k = left_camera_matrix;
    // camera_info_left_.r = left_rectification_matrix;
    // camera_info_left_.p = left_projection_matrix;

    pub_info_left_->publish(camera_info_left_);
    pub_info_right_->publish(camera_info_right_);

  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InfoPublisher>());
  rclcpp::shutdown();
  return 0;
}