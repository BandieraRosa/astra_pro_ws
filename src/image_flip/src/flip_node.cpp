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
// Horizontal-flip relay: republishes image + camera_info in the
// unmirrored domain. Under x' = W - 1 - x the pinhole/distortion map is:
//   cx' = W - 1 - cx, p2' = -p2 (d[3]), everything else unchanged.
// Applies to plumb_bob (5 coeff) and rational_polynomial (8 coeff),
// whose d ordering shares the k1,k2,p1,p2,k3 prefix.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "image_flip/camera_info_flip.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

int encoding_channels(const std::string & encoding, int & cv_type)
{
  if (encoding == "mono8") {
    cv_type = CV_8UC1;
    return 1;
  }
  if (encoding == "rgb8" || encoding == "bgr8") {
    cv_type = CV_8UC3;
    return 3;
  }
  if (encoding == "mono16" || encoding == "16UC1") {
    cv_type = CV_16UC1;
    return 1;
  }
  cv_type = -1;
  return 0;
}

}  // namespace

class ImageFlip : public rclcpp::Node
{
public:
  explicit ImageFlip(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("image_flip", options)
  {
    input_image_ = declare_parameter<std::string>("input_image", "/camera/color/image_raw");
    input_info_ = declare_parameter<std::string>("input_info", "/camera/color/camera_info");
    output_image_ = declare_parameter<std::string>("output_image", "/camera/color/image_flipped");
    output_info_ =
      declare_parameter<std::string>("output_info", "/camera/color/camera_info_flipped");

    const rclcpp::QoS qos = rclcpp::SensorDataQoS();
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      input_image_, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {onImage(msg);});
    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      input_info_, qos,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {info_ = *msg;});
    pub_image_ = create_publisher<sensor_msgs::msg::Image>(output_image_, qos);
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_info_, qos);
    RCLCPP_INFO(get_logger(), "flip %s -> %s", input_image_.c_str(), output_image_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    int cv_type = -1;
    const int channels = encoding_channels(msg->encoding, cv_type);
    if (channels == 0 || static_cast<size_t>(msg->height) * msg->step != msg->data.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "unsupported image (%s %ux%u)",
        msg->encoding.c_str(), msg->width, msg->height);
      return;
    }
    cv::Mat src(msg->height, msg->width, cv_type, const_cast<uint8_t *>(msg->data.data()),
      msg->step);
    cv::Mat flipped;
    cv::flip(src, flipped, 1);

    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = msg->header;
    out->height = msg->height;
    out->width = msg->width;
    out->encoding = msg->encoding;
    out->is_bigendian = msg->is_bigendian;
    out->step = static_cast<uint32_t>(flipped.cols * channels *
      static_cast<int>(flipped.elemSize1()));
    out->data.assign(flipped.data, flipped.data + flipped.total() * flipped.elemSize());

    sensor_msgs::msg::CameraInfo info = info_;
    info.header.stamp = msg->header.stamp;
    if (info.width == msg->width && info.height == msg->height) {
      info = image_flip::horizontallyFlipped(std::move(info));
    }
    pub_image_->publish(std::move(out));
    pub_info_->publish(info);
  }

  std::string input_image_;
  std::string input_info_;
  std::string output_image_;
  std::string output_info_;
  sensor_msgs::msg::CameraInfo info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ImageFlip)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageFlip>());
  rclcpp::shutdown();
  return 0;
}
