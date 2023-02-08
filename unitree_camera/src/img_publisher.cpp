#include <memory>
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
    param.description = "The frame rate of the camera (fps).";
    declare_parameter("fps", 30, param);
    fps_ = get_parameter("fps").get_parameter_value().get<int>();
    interval_ = 1.0 / static_cast<double>(fps_);
    interval_ms_ = static_cast<std::chrono::milliseconds>(static_cast<int>(interval_ * 1000.0));

    param.description = "Camera device node number.";
    declare_parameter("device_node", 0, param);
    device_node_ = get_parameter("device_node").get_parameter_value().get<int>();

    param.description = "Camera frame width.";
    declare_parameter("frame_width", 1856, param);
    frame_width_ = get_parameter("frame_width").get_parameter_value().get<int>();

    param.description = "Camera frame height.";
    declare_parameter("frame_height", 800, param);
    frame_height_ = get_parameter("frame_height").get_parameter_value().get<int>();

    //Timers
    timer_ = create_wall_timer(interval_ms_, std::bind(&ImgPublisher::timer_callback, this));

    frame_size_ = cv::Size {frame_width_, frame_height_};

    //Initialize camera
    cam_ = std::make_unique<UnitreeCamera>(device_node_);

    if (!cam_->isOpened()) {
      //TODO - exit if camera fails to open
    }

    //TODO make all things optional based on parameters
    cam_->setRawFrameSize(frame_size_);
    cam_->setRawFrameRate(fps_);

    RCLCPP_INFO_STREAM(get_logger(), "Device Position Number: " << cam_->getPosNumber());

    //Start camera capturing
    cam_->startCapture();

    RCLCPP_INFO_STREAM(get_logger(), "img_publisher node started");
  }

  //Stop capture when node is destroyed
  ~ImgPublisher()
  {
    cam_->stopCapture();
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;

  double interval_;
  std::chrono::milliseconds interval_ms_;
  int fps_, device_node_, frame_width_, frame_height_;
  cv::Size frame_size_ {1856, 800};

  std::unique_ptr<UnitreeCamera> cam_;

  void timer_callback()
  {
    if (!cam_->isOpened()) {
      //TODO - exit if camera is no longer open
    }

    std::chrono::microseconds t;

    cv::Mat raw_frame;
    //Process/publish raw frame if it can be obtained
    if (cam_->getRawFrame(raw_frame, t)) {
      cv::Mat raw_left, raw_right;
      
      //Get left and right images from returned frame
      raw_frame(
        cv::Rect(0, 0, raw_frame.size().width/2, raw_frame.size().height)
      ).copyTo(raw_right);
      raw_frame(
        cv::Rect(raw_frame.size().width/2,0, raw_frame.size().width/2, raw_frame.size().height)
      ).copyTo(raw_left);
    }

  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgPublisher>());
  rclcpp::shutdown();
  return 0;
}