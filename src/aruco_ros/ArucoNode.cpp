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

#include <list>

#include "aruco_ros/ArucoNode.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "rclcpp/rclcpp.hpp"

namespace aruco_ros
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ArucoNode::ArucoNode(const rclcpp::NodeOptions & options)
: Node("aruco_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "camera1/image_raw", 10, std::bind(&ArucoNode::img_callback, this, _1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera1/camera_info", 10, std::bind(&ArucoNode::info_callback, this, _1));
    aruco_pub_ = create_publisher<sensor_msgs::msg::Image>("camera1/aruco", 10);
}

void
ArucoNode::img_callback(
    const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    // Image callback  
    cv::Mat image = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs, _objpoints;

    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    RCLCPP_INFO(get_logger(), "ArUCos detected %ld", ids.size());
    // If there are markers
    
    if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, intrinsec_mat_, distortion_, rvecs, tvecs);

        // Ahora, rvecs y tvecs contienen la rotación y la traslación de los marcadores respectivamente.
        for (size_t i = 0; i < ids.size(); ++i)
        {
            std::cout << "Marker " << ids[i] << ": tvec = " << tvecs[i] << ", rvec = " << rvecs[i] << std::endl;
        }
        cv::aruco::drawDetectedMarkers(image, corners, ids);
        // Dibuja ejes para cada marcador detectado
        for (int i = 0; i < (int)ids.size(); i++) {
            RCLCPP_INFO(get_logger(), "Dibujando frame num: %i", i);
            RCLCPP_INFO(get_logger(), "Valores: %s", tvecs.at(0));
            std::cout << "Valores cout: " << tvecs.data() << std::endl;
            cv::drawFrameAxes(image, intrinsec_mat_, distortion_, rvecs[i], tvecs[i], 1.5, 2);
            std::cout << "Valores rvecs: " << rvecs[i] << std::endl;
        } 

    }
    publish_img(image);
        
}

void
ArucoNode::info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr info_msg)
{
    // Camera info callback
    intrinsec_mat_ = cv::Mat(3, 3, CV_64F, &info_msg->k);
    distortion_ = cv::Mat(1, 5, CV_64F, &info_msg->d);
    std::cout << "Intrinsec parameters = " << std::endl << " " << intrinsec_mat_ << std::endl << std::endl;
    std::cout << "Distortion parameters = " << std::endl << " " << distortion_ << std::endl << std::endl;
    RCLCPP_INFO(get_logger(), "Camera info received");
    info_sub_ = nullptr;
}

void
ArucoNode::publish_img(
   cv::Mat & img)
{
    img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img)
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    aruco_pub_->publish(*img_msg_.get());
    // Publish image
    // RCLCPP_INFO(get_logger(), "Image published");
}

}  // namespace aruco_ros