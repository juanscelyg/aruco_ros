// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARUCO_ROS__ARUCONODE_HPP_
#define ARUCO_ROS__ARUCONODE_HPP_

#include <string>
#include <memory>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"

#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "rclcpp/rclcpp.hpp"

namespace aruco_ros
{

class ArucoNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ArucoNode)

  explicit ArucoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_img(
     cv::Mat & img);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aruco_pub_;

  sensor_msgs::msg::Image::SharedPtr last_img_; 
  sensor_msgs::msg::Image::SharedPtr img_msg_;


  void img_callback(sensor_msgs::msg::Image::SharedPtr img_msg);
  void info_callback(sensor_msgs::msg::CameraInfo::SharedPtr info_msg);

  std::string camera_frame_id_ {"camera_frame"};
  std::string robot_frame_id_ {"base_link"};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cv::Mat intrinsec_mat_;
  cv::Mat distortion_;

  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
  double marker_size_ {0.1};

};

}  // namespace aruco_ros

#endif  // ARUCO_ROS__ARUCONODE_HPP_