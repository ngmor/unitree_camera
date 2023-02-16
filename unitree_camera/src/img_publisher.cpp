#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "UnitreeCameraSDK.hpp"
#include "cv_bridge/cv_bridge.h"

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
    int frame_width = get_parameter("frame_width").get_parameter_value().get<int>();

    param.description = "Camera frame height.";
    declare_parameter("frame_height", 800, param);
    int frame_height = get_parameter("frame_height").get_parameter_value().get<int>();

    param.description = "Enable publishing of raw frames.";
    declare_parameter("enable_raw", false, param);
    enable_raw_ = get_parameter("enable_raw").get_parameter_value().get<bool>();

    param.description = "Enable publishing of rectified frames.";
    declare_parameter("enable_rect", true, param);
    enable_rect_ = get_parameter("enable_rect").get_parameter_value().get<bool>();

    param.description = "Enable publishing of depth frames.";
    declare_parameter("enable_depth", false, param);
    enable_depth_ = get_parameter("enable_depth").get_parameter_value().get<bool>();

    //Timers
    timer_ = create_wall_timer(interval_ms_, std::bind(&ImgPublisher::timer_callback, this));

    frame_size_ = cv::Size {frame_width, frame_height};

    //Initialize camera
    //TODO - need to initialize with config yaml?
    //Doesn't seem to be necessary for rectification
    cam_ = std::make_unique<UnitreeCamera>(device_node_);

    if (!cam_->isOpened()) {
      //TODO - exit if camera fails to open
    }

    //Set frame size and fps
    cam_->setRawFrameSize(frame_size_);
    cam_->setRawFrameRate(fps_);

    if (enable_raw_) {
      pub_raw_left_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
      pub_raw_right_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
    }

    if (enable_rect_) {
      cam_->setRectFrameSize(cv::Size(frame_size_.width >> 2, frame_size_.height >> 1));

      pub_rect_left_ = create_publisher<sensor_msgs::msg::Image>("left/image_rect", 10);
      pub_rect_right_ = create_publisher<sensor_msgs::msg::Image>("right/image_rect", 10);
    }

    if (enable_depth_) {
      cam_->startStereoCompute();

      pub_depth_ = create_publisher<sensor_msgs::msg::Image>("image_depth", 10);
    }

    RCLCPP_INFO_STREAM(get_logger(), "Device Position Number: " << cam_->getPosNumber());

    //Start camera capturing
    cam_->startCapture();

    RCLCPP_INFO_STREAM(get_logger(), "img_publisher node started");
  }

  //Stop capture when node is destroyed
  ~ImgPublisher()
  {
    if (enable_depth_) {
      cam_->stopStereoCompute();
    }
    cam_->stopCapture();
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rect_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rect_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;

  double interval_;
  std::chrono::milliseconds interval_ms_;
  int fps_, device_node_;
  bool enable_raw_, enable_rect_, enable_depth_;
  cv::Size frame_size_ {1856, 800};
  const std::string color_encoding_ = "bgr8"; //TODO allow to change?
  const std::string depth_encoding_ = "8UC3";

  std::unique_ptr<UnitreeCamera> cam_;

  void timer_callback()
  {
    if (!cam_->isOpened()) {
      //TODO - exit if camera is no longer open
    }

    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();

    std::chrono::microseconds t;
    
    //Get and publish raw frames
    if (enable_raw_) {
      cv::Mat raw_frame;

      //Process/publish raw frame if it can be obtained
      if (cam_->getRawFrame(raw_frame, t)) {
        cv::Mat raw_left, raw_right;
        
        //Get left and right images from returned frame
        raw_frame(
          cv::Rect(raw_frame.size().width/2,0, raw_frame.size().width/2, raw_frame.size().height)
        ).copyTo(raw_left);
        raw_frame(
          cv::Rect(0, 0, raw_frame.size().width/2, raw_frame.size().height)
        ).copyTo(raw_right);

        //Publish frames
        pub_raw_left_->publish(*(cv_bridge::CvImage(header, color_encoding_, raw_left).toImageMsg()));
        pub_raw_right_->publish(*(cv_bridge::CvImage(header, color_encoding_, raw_right).toImageMsg()));
      }
    }

    //Get and publish rectified frames
    if (enable_rect_) {
      cv::Mat rect_left, rect_right;

      if(cam_->getRectStereoFrame(rect_left, rect_right)) {
        //flip frames
        cv::flip(rect_left, rect_left, -1);
        cv::flip(rect_right, rect_right, -1);

        //Publish frames
        pub_rect_left_->publish(*(cv_bridge::CvImage(header, color_encoding_, rect_left).toImageMsg()));
        pub_rect_right_->publish(*(cv_bridge::CvImage(header, color_encoding_, rect_right).toImageMsg()));
      }
    }

    //Get and publish depth frames
    if(enable_depth_) {
      cv::Mat depth;

      if(cam_->getDepthFrame(depth, true, t)) {
        if(!depth.empty()) {
          //Publish frames
          pub_depth_->publish(*(cv_bridge::CvImage(header, depth_encoding_, depth).toImageMsg()));
        }
      }
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