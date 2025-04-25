#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_geometry/pinhole_camera_model.h>

class HumanPoseDetector : public rclcpp::Node {
public:
  HumanPoseDetector()
    : Node("human_pose_detector"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
    
    using std::placeholders::_1;

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10,
        std::bind(&HumanPoseDetector::image_callback, this, _1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw", 10,
        std::bind(&HumanPoseDetector::depth_callback, this, _1));

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", 10,
        std::bind(&HumanPoseDetector::cam_info_callback, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/human_goal_pose", 10);

    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
  }

private:
  void cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    cam_model_.fromCameraInfo(msg);
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_depth_msg_ = msg;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!last_depth_msg_) return;

    cv::Mat color_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat depth_img = cv_bridge::toCvCopy(last_depth_msg_, "16UC1")->image;

    std::vector<cv::Rect> detections;
    hog_.detectMultiScale(color_img, detections);
    if (detections.empty()) return;

    cv::Rect human = detections[0];
    cv::Point center(human.x + human.width / 2, human.y + human.height / 2);
    float depth = depth_img.at<uint16_t>(center) / 1000.0f;  // mm → m

    if (depth == 0) return;  // Invalid depth

    cv::Point3d ray = cam_model_.projectPixelTo3dRay(center);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = cam_model_.tfFrame();  // usually "camera_color_optical_frame"
    pose_msg.pose.position.x = ray.x * depth;
    pose_msg.pose.position.y = ray.y * depth;
    pose_msg.pose.position.z = ray.z * depth;
    pose_msg.pose.orientation.w = 1.0;

    // Transform to "map"
    try {
      auto transformed_pose = tf_buffer_.transform(pose_msg, "map", tf2::durationFromSec(0.1));
      pose_pub_->publish(transformed_pose);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  sensor_msgs::msg::Image::SharedPtr last_depth_msg_;
  image_geometry::PinholeCameraModel cam_model_;

  cv::HOGDescriptor hog_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
