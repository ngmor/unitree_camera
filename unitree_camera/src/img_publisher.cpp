#include "rclcpp/rclcpp.hpp"
#include "UnitreeCameraSDK.hpp"

using namespace std::chrono_literals;

class ImgPublisher : public rclcpp::Node
{
public:
  ImgPublisher()
  : Node("img_publisher")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The rate at which the node publishes images (Hz).";
    declare_parameter("rate", 60.0, param);
    rate_ = get_parameter("rate").get_parameter_value().get<double>();
    interval_ = 1.0 / rate_;
    interval_ms_ = static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0));

    //Timers
    timer_ = create_wall_timer(interval_ms_, std::bind(&ImgPublisher::timer_callback, this));


    RCLCPP_INFO_STREAM(get_logger(), "img_publisher node started");
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;

  double rate_, interval_;
  std::chrono::milliseconds interval_ms_;

  UnitreeCamera test;

  void timer_callback()
  {

  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgPublisher>());
  rclcpp::shutdown();
  return 0;
}