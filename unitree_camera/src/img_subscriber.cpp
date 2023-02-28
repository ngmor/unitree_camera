//This node is intended to be used for testing/example of subscribing
// to images published by img_publisher.
//TODO document
#include <string>
#include <string_view>
#include <regex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "image_transport/image_transport.hpp"

const std::string RAW_LEFT_WINDOW = " raw left image";
const std::string RAW_RIGHT_WINDOW = " raw right image";
const std::string RECT_LEFT_WINDOW = " rectified left image";
const std::string RECT_RIGHT_WINDOW = " rectified right image";
const std::string DEPTH_WINDOW = " depth image";

void show_image(const std::string & window, const sensor_msgs::msg::Image::ConstSharedPtr& img);

class ImgSubscriber : public rclcpp::Node
{
public:
  ImgSubscriber()
  : Node("img_subscriber"),
    prefix_{std::regex_replace(get_namespace(), std::regex("/"), " ")},
    raw_left_window_{prefix_ + RAW_LEFT_WINDOW},
    raw_right_window_{prefix_ + RAW_RIGHT_WINDOW},
    rect_left_window_{prefix_ + RECT_LEFT_WINDOW},
    rect_right_window_{prefix_ + RECT_RIGHT_WINDOW},
    depth_window_{prefix_ + DEPTH_WINDOW}
  {
    //Subscribers
    sub_raw_left_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_raw/left",
          std::bind(&ImgSubscriber::raw_left_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );
    sub_raw_right_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_raw/right",
          std::bind(&ImgSubscriber::raw_right_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );
    sub_rect_left_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_rect/left",
          std::bind(&ImgSubscriber::rect_left_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );
    sub_rect_right_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "image_rect/right",
          std::bind(&ImgSubscriber::rect_right_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );
    sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
      "image_depth",
      10,
      std::bind(&ImgSubscriber::depth_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO_STREAM(get_logger(), "img_subscriber node started");

  }

  //Destroy windows
  ~ImgSubscriber()
  {
    cv::destroyAllWindows();
  }

private:
  std::shared_ptr<image_transport::CameraSubscriber> sub_raw_left_;
  std::shared_ptr<image_transport::CameraSubscriber> sub_raw_right_;
  std::shared_ptr<image_transport::CameraSubscriber> sub_rect_left_;
  std::shared_ptr<image_transport::CameraSubscriber> sub_rect_right_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;

  const std::string prefix_ = "";
  const std::string raw_left_window_ = "";
  const std::string raw_right_window_ = "";
  const std::string rect_left_window_ = "";
  const std::string rect_right_window_ = "";
  const std::string depth_window_ = "";

  void raw_left_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    show_image(raw_left_window_, img);
  }

  void raw_right_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    show_image(raw_right_window_, img);
  }

  void rect_left_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    show_image(rect_left_window_, img);
  }

  void rect_right_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    show_image(rect_right_window_, img);
  }

  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
    show_image(depth_window_, img);
  }


};

void show_image(const std::string & window, const sensor_msgs::msg::Image::ConstSharedPtr& img) {
  cv::namedWindow(window);
  cv::imshow(window, cv_bridge::toCvCopy(*img, img->encoding)->image);
  cv::waitKey(1);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgSubscriber>());
  rclcpp::shutdown();
  return 0;
}