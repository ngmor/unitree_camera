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

    param.description = "name for the stereo pair";
    declare_parameter("stereo_name", "stereo", param);
    stereo_name = get_parameter("stereo_name").get_parameter_value().get<std::string>();

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


    param.description = "right camera frame width";
    declare_parameter("right_image_width", 0, param);
    right_image_width = get_parameter("right_image_width").get_parameter_value().get<int>();

    param.description = "right camera frame height";
    declare_parameter("right_image_height", 0, param);
    right_image_height = get_parameter("right_image_height").get_parameter_value().get<int>();

    param.description = "right Camera name";
    declare_parameter("right_camera_name", "", param);
    right_camera_name = get_parameter("right_camera_name").get_parameter_value().get<std::string>();

    param.description = "right Camera matrix";
    declare_parameter("right_camera_matrix", std::vector<double>{}, param);
    right_camera_matrix = get_parameter("right_camera_matrix").get_parameter_value().get<std::vector<double>>();

    param.description = "right Camera distortion model";
    declare_parameter("right_distortion_model", "", param);
    right_distortion_model = get_parameter("right_distortion_model").get_parameter_value().get<std::string>();

    param.description = "right Camera distortion coefficients";
    declare_parameter("right_distortion_coefficients", std::vector<double>{}, param);
    right_distortion_coefficients = get_parameter("right_distortion_coefficients").get_parameter_value().get<std::vector<double>>();

    param.description = "right Camera rectification matrix";
    declare_parameter("right_rectification_matrix", std::vector<double>{}, param);
    right_rectification_matrix = get_parameter("right_rectification_matrix").get_parameter_value().get<std::vector<double>>();

    param.description = "right Camera projection matrix matrix";
    declare_parameter("right_projection_matrix", std::vector<double>{}, param);
    right_projection_matrix = get_parameter("right_projection_matrix").get_parameter_value().get<std::vector<double>>();

    // Publisher
    pub_info_left_ = create_publisher<sensor_msgs::msg::CameraInfo>(stereo_name+"/cam/image_rect/left/camera_info", 10);
    pub_info_right_ = create_publisher<sensor_msgs::msg::CameraInfo>(stereo_name+"/cam/image_rect/right/camera_info", 10);
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

  std::string stereo_name;

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

  bool fields_set = false;

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

    if (!fields_set){
      camera_info_left_.height = left_image_height;
      camera_info_left_.width = left_image_width;
      camera_info_left_.distortion_model = left_distortion_model;
      camera_info_left_.d = left_distortion_coefficients;

      camera_info_right_.height = right_image_height;
      camera_info_right_.width = right_image_width;
      camera_info_right_.distortion_model = right_distortion_model;
      camera_info_right_.d = right_distortion_coefficients;
      // these following ones have set sizes so I might need to fill in manually ;-;
      // there is definitely more sophisticated way but I am in dumb brain mode rn
      for (int i = 0; i < 9; i++){
        camera_info_left_.k.at(i) = left_camera_matrix.at(i);
        camera_info_left_.r.at(i) = left_rectification_matrix.at(i);

        camera_info_right_.k.at(i) = right_camera_matrix.at(i);
        camera_info_right_.r.at(i) = right_rectification_matrix.at(i);
      }
      for (int i = 0; i < 12; i++){
        camera_info_left_.p.at(i) = left_projection_matrix.at(i);

        camera_info_right_.p.at(i) = right_projection_matrix.at(i);
      }
      fields_set = true;
    }

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