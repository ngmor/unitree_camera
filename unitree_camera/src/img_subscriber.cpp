//This node is intended to be used for testing/example of subscribing
// to images published by img_publisher.
//TODO document
#include <string>
#include <string_view>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include "image_transport/image_transport.hpp"

const std::string RAW_LEFT_WINDOW = "Raw Left Image";
const std::string RAW_RIGHT_WINDOW = "Raw Right Image";
const std::string RECT_LEFT_WINDOW = "Rectified Left Image";
const std::string RECT_RIGHT_WINDOW = "Rectified Right Image";
const std::string DEPTH_WINDOW = "Depth";

class ImgSubscriber : public rclcpp::Node
{
public:
  ImgSubscriber()
  : Node("img_subscriber")
  {
    //Subscribers
    sub_raw_left_ = create_subscription<sensor_msgs::msg::Image>(
      "left/image_raw",
      10,
      std::bind(&ImgSubscriber::raw_left_callback, this, std::placeholders::_1)
    );
    sub_raw_right_ = create_subscription<sensor_msgs::msg::Image>(
      "right/image_raw",
      10,
      std::bind(&ImgSubscriber::raw_right_callback, this, std::placeholders::_1)
    );
    sub_rect_left_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "left/image_rect",
          std::bind(&ImgSubscriber::rect_left_callback, this, std::placeholders::_1, std::placeholders::_2),
          "compressed",
          rclcpp::QoS {10}.get_rmw_qos_profile()
        )
    );
    sub_rect_right_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this,
          "right/image_rect",
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

    cv::namedWindow(RAW_LEFT_WINDOW);
    cv::namedWindow(RAW_RIGHT_WINDOW);
    cv::namedWindow(RECT_LEFT_WINDOW);
    cv::namedWindow(RECT_RIGHT_WINDOW);
    cv::namedWindow(DEPTH_WINDOW);

    RCLCPP_INFO_STREAM(get_logger(), "img_subscriber node started");

  }

  //Destroy windows
  ~ImgSubscriber()
  {
    cv::destroyAllWindows();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_right_;
  std::shared_ptr<image_transport::CameraSubscriber> sub_rect_left_;
  std::shared_ptr<image_transport::CameraSubscriber> sub_rect_right_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;

  void raw_left_callback(const sensor_msgs::msg::Image & msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
    cv::imshow(RAW_LEFT_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }

  void raw_right_callback(const sensor_msgs::msg::Image & msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
    cv::imshow(RAW_RIGHT_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }

  void rect_left_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*img, img->encoding);
    cv::imshow(RECT_LEFT_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }

  void rect_right_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
  ) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*img, img->encoding);
    cv::imshow(RECT_RIGHT_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }

  void depth_callback(const sensor_msgs::msg::Image & msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
    cv::imshow(DEPTH_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }


};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgSubscriber>());
  rclcpp::shutdown();
  return 0;
}