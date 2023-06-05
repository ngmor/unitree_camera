//TODO document

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

std::shared_ptr<sensor_msgs::msg::PointCloud2> unitree_pcl_to_ros_msg(
  const std::vector<PCLType> & pcl,
  const std_msgs::msg::Header & header,
  const std::string_view frame_id
);

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

    param.description = "Camera frame width";
    declare_parameter("image_width", 0, param);
    image_width = get_parameter("image_width").get_parameter_value().get<int>();

    param.description = "Camera frame height";
    declare_parameter("image_height", 0, param);
    image_height = get_parameter("image_height").get_parameter_value().get<int>();

    param.description = "Camera name";
    declare_parameter("camera_name", "", param);
    camera_name = get_parameter("camera_name").get_parameter_value().get<std::string>();

    // Publisher
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("infopls", 10);
    // Timer
    timer_ = create_wall_timer(interval_ms_, std::bind(&InfoPublisher::timer_callback, this));

  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;

  double interval_;
  std::chrono::milliseconds interval_ms_;
  int fps_;

  // stuff I added
  int image_width, image_height;
  std::string camera_name;

  sensor_msgs::msg::CameraInfo camera_info_;

  void timer_callback()
  {

    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    camera_info_.header = header;

    // things to set:
    // camera_info_.height
    // camera_info_.width
    // camera_info_.distortion_model
    // camera_info_.d
    // camera_info_.k
    // camera_info_.r
    // camera_info_.p

    pub_info_->publish(camera_info_);

  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InfoPublisher>());
  rclcpp::shutdown();
  return 0;
}