// Copyright 2026 BandieraRossa
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

#ifndef STEREO_D2C_CALIB__CAPTURE_NODE_HPP_
#define STEREO_D2C_CALIB__CAPTURE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace stereo_d2c_calib
{

class D2CCapture : public rclcpp::Node
{
public:
  explicit D2CCapture(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  static bool findBoard(
    const cv::Mat & gray, int cols, int rows,
    std::vector<cv::Point2f> & corners);
  static bool diverse(
    const std::vector<cv::Point2f> & corners,
    const std::vector<cv::Point2f> & prev);
  void publishSide(
    rclcpp::Publisher<Image>::SharedPtr pub, cv::Mat bgr,
    const std::vector<cv::Point2f> & corners, bool ok, const char * tag,
    size_t saved, const std_msgs::msg::Header & header);
  void callback(const Image::ConstSharedPtr & ir_msg, const Image::ConstSharedPtr & color_msg);

  std::string out_dir_, ir_topic_, color_topic_;
  int cols_, rows_;
  double square_, auto_interval_;
  size_t saved_ = 0;
  double last_save_t_ = 0.0;
  std::vector<cv::Point2f> prev_ir_, prev_color_;
  message_filters::Subscriber<Image> sub_ir_, sub_color_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<Image>::SharedPtr debug_ir_pub_, debug_color_pub_;
};

}  // namespace stereo_d2c_calib

#endif  // STEREO_D2C_CALIB__CAPTURE_NODE_HPP_
