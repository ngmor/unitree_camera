//TODO document

#include <memory>
#include <string>
#include <vector>
#include <exception>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "UnitreeCameraSDK.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace std::chrono_literals;

std::shared_ptr<sensor_msgs::msg::PointCloud2> unitree_pcl_to_ros_msg(
  const std::vector<PCLType> & pcl,
  const std_msgs::msg::Header & header,
  const std::string_view frame_id
);

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
    int device_node = get_parameter("device_node").get_parameter_value().get<int>();

    param.description = "Camera frame width.";
    declare_parameter("frame_width", 1856, param);
    int frame_width = get_parameter("frame_width").get_parameter_value().get<int>();

    param.description = "Camera frame height.";
    declare_parameter("frame_height", 800, param);
    int frame_height = get_parameter("frame_height").get_parameter_value().get<int>();

    param.description = "Configure camera with YAML file instead of parameters. "
                        "MUST be true if using point cloud.";
    declare_parameter("use_yaml", false, param);
    auto use_yaml = get_parameter("use_yaml").get_parameter_value().get<bool>();

    param.description = "Path to yaml configuration file.";
    declare_parameter("yaml_path", "", param);
    auto yaml_path = get_parameter("yaml_path").get_parameter_value().get<std::string>();

    param.description = "Enable publishing of raw frames.";
    declare_parameter("enable_raw", false, param);
    enable_raw_ = get_parameter("enable_raw").get_parameter_value().get<bool>();

    param.description = "Enable publishing of rectified frames.";
    declare_parameter("enable_rect", true, param);
    enable_rect_ = get_parameter("enable_rect").get_parameter_value().get<bool>();

    param.description = "Enable publishing of depth frames.";
    declare_parameter("enable_depth", false, param);
    enable_depth_ = get_parameter("enable_depth").get_parameter_value().get<bool>();

    param.description = "Enable publishing point cloud data.";
    declare_parameter("enable_point_cloud", false, param);
    enable_point_cloud_ = get_parameter("enable_point_cloud").get_parameter_value().get<bool>();

    param.description = "Frame ID for point cloud messages.";
    declare_parameter("point_cloud_frame", "map", param);
    point_cloud_frame_ = get_parameter("point_cloud_frame").get_parameter_value().get<std::string>();

    //Throw exception if invalid inputs are provided
    
    if (enable_point_cloud_ && (!use_yaml)) {
      std::string msg = "Cannot activate point cloud without using yaml file for configuration.";

      RCLCPP_ERROR_STREAM(get_logger(), msg);
      throw std::logic_error(msg);
    }

    if (use_yaml && (yaml_path == "")) {
      std::string msg = "Must specify yaml configuration file path to use one.";

      RCLCPP_ERROR_STREAM(get_logger(), msg);
      throw std::logic_error(msg);
    }

    //Timers
    timer_ = create_wall_timer(interval_ms_, std::bind(&ImgPublisher::timer_callback, this));

    frame_size_ = cv::Size {frame_width, frame_height};

    //Initialize camera
    if (!use_yaml) {
      cam_ = std::make_unique<UnitreeCamera>(device_node);
    } else {
      cam_ = std::make_unique<UnitreeCamera>(yaml_path);
    }

    //Throw error and exit if camera fails to open
    if (!cam_->isOpened()) {
      std::string msg = "Camera failed to open on startup.";

      RCLCPP_ERROR_STREAM(get_logger(), msg);
      throw std::logic_error(msg);
    }
    

    //Set frame size and fps
    if (!use_yaml) {
      cam_->setRawFrameSize(frame_size_);
      cam_->setRawFrameRate(fps_);
    }

    if (enable_raw_) {
      pub_raw_left_ = create_publisher<sensor_msgs::msg::Image>("~/left/image_raw", 10);
      pub_raw_right_ = create_publisher<sensor_msgs::msg::Image>("~/right/image_raw", 10);
    }

    if (enable_rect_) {
      if (!use_yaml) {
        cam_->setRectFrameSize(cv::Size(frame_size_.width >> 2, frame_size_.height >> 1));
      }

      pub_rect_left_ = create_publisher<sensor_msgs::msg::Image>("~/left/image_rect", 10);
      pub_rect_right_ = create_publisher<sensor_msgs::msg::Image>("~/right/image_rect", 10);
    }

    if (enable_depth_) {
      pub_depth_ = create_publisher<sensor_msgs::msg::Image>("~/image_depth", 10);
    }
    if (enable_point_cloud_) {
      pub_point_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud", 10);
    }

    RCLCPP_INFO_STREAM(get_logger(), "Device Position Number: " << cam_->getPosNumber());

    //Start camera capturing
    
    cam_->startCapture();
    if (enable_depth_ || enable_point_cloud_) {cam_->startStereoCompute();}

    RCLCPP_INFO_STREAM(get_logger(), "img_publisher node started");
  }

  //Stop capture when node is destroyed
  ~ImgPublisher()
  {
    if (enable_depth_ || enable_point_cloud_) {cam_->stopStereoCompute();}
    cam_->stopCapture();
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rect_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rect_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;

  double interval_;
  std::chrono::milliseconds interval_ms_;
  int fps_;
  bool enable_raw_, enable_rect_, enable_depth_, enable_point_cloud_;
  std::string point_cloud_frame_;
  cv::Size frame_size_ {1856, 800};
  const std::string color_encoding_ = "bgr8";
  const std::string depth_encoding_ = "8UC3";

  std::unique_ptr<UnitreeCamera> cam_;

  void timer_callback()
  {
    //Throw error and exit if camera is no longer open
    if (!cam_->isOpened()) {
      std::string msg = "Camera closed unexpectedly.";

      RCLCPP_ERROR_STREAM(get_logger(), msg);
      throw std::logic_error(msg);
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

    //Get and publish point cloud data
    if (enable_point_cloud_) {
      std::vector<PCLType> point_cloud;

      if(cam_->getPointCloud(point_cloud, t)) {
        //Convert to PointCloud2 message and publish
        auto msg = unitree_pcl_to_ros_msg(point_cloud, header, point_cloud_frame_);
        pub_point_cloud_->publish(*msg);
      }
    }

  }

};

//Convert Unitree PCL to ros message
std::shared_ptr<sensor_msgs::msg::PointCloud2> unitree_pcl_to_ros_msg(
  const std::vector<PCLType> & pcl,
  const std_msgs::msg::Header & header,
  const std::string_view frame_id
)
{
  //This function is loosely based on this answers.ros.org post
  //https://answers.ros.org/question/312587/generate-and-publish-pointcloud2-in-ros2/
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  for (const auto & point : pcl) {
    //Construct a PCL XYZRGB point out of the point provided by the Unitree PCLType
    //and append to points list in point cloud
    //x axis is aligned with the direction of the camera
    //y axis is pointing towards the left of the camera
    //z axis is pointing up from the camera's perspective
    cloud.points.push_back(pcl::PointXYZRGB {
      point.pts(2),   //x
      -point.pts(0),  //y
      -point.pts(1),  //z
      point.clr(2),   //r
      point.clr(1),   //g
      point.clr(0)    //b
    });
  }

  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *msg);
  msg->header = header;
  msg->header.frame_id = frame_id;

  return msg;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgPublisher>());
  rclcpp::shutdown();
  return 0;
}