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

#include "ir_converter/ir_converter.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace ir_converter
{

IrY10Converter::IrY10Converter(const rclcpp::NodeOptions & options)
: Node("ir_converter", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/camera/ir/image_raw");
  output_topic_ =
    declare_parameter<std::string>("output_topic", "/camera/ir/image_mono8");
  // linear: fixed >>2 map (predictable); normalize: per-frame min-max stretch (brighter)
  mode_ = declare_parameter<std::string>("mode", "linear");
  const auto qos_profile = declare_parameter<std::string>("qos_profile", "sensor_data");
  const auto qos_depth = declare_parameter<int>("qos_depth", 10);

  const auto qos = makeQoS(qos_profile, qos_depth);

  sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_, qos,
    std::bind(&IrY10Converter::callback, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, qos);

  RCLCPP_INFO(
    get_logger(),
    "Y10/mono16 -> mono8 converter started: %s -> %s (qos=%s, depth=%d)",
    input_topic_.c_str(), output_topic_.c_str(), qos_profile.c_str(),
    static_cast<int>(qos_depth));
}

rclcpp::QoS IrY10Converter::makeQoS(const std::string & profile, int depth)
{
  const int history_depth = depth > 0 ? depth : 10;
  if (profile == "default") {
    return rclcpp::QoS(history_depth);
  }
  if (profile == "system_default") {
    return rclcpp::SystemDefaultsQoS();
  }
  return rclcpp::SensorDataQoS();
}

void IrY10Converter::callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (msg->encoding != "mono16") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Expected mono16, got: %s",
      msg->encoding.c_str());
    return;
  }

  auto out = std::make_unique<sensor_msgs::msg::Image>();

  out->header = msg->header;
  out->height = msg->height;
  out->width = msg->width;
  out->encoding = "mono8";
  out->is_bigendian = false;
  out->step = msg->width;
  out->data.resize(static_cast<size_t>(out->height) * out->step);

  uint16_t min_value = std::numeric_limits<uint16_t>::max();

  uint16_t max_value = 0;

  for (uint32_t y = 0; y < msg->height; ++y) {
    const uint8_t * row = msg->data.data() + static_cast<size_t>(y) * msg->step;

    for (uint32_t x = 0; x < msg->width; ++x) {
      uint16_t value;

      std::memcpy(&value, row + x * sizeof(uint16_t), sizeof(uint16_t));

      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);

      // Astra Pro Y10:
      // 10-bit -> 8-bit
      uint16_t scaled = value >> 2;

      if (scaled > 255) {
        scaled = 255;
      }

      out->data[static_cast<size_t>(y) * out->step + x] = static_cast<uint8_t>(scaled);
    }
  }

  if (mode_ == "normalize") {
    const auto lo = static_cast<int>(min_value >> 2);
    const auto hi = static_cast<int>(max_value >> 2);
    if (hi > lo) {
      const float gain = 255.0f / static_cast<float>(hi - lo);
      for (auto & px : out->data) {
        // out.data currently holds value>>2; stretch that domain to full range
        float v = (static_cast<float>(px) - static_cast<float>(lo)) * gain;
        px = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
      }
    }
  }

  cv::Mat mono8(out->height, out->width, CV_8UC1, out->data.data(), out->step);

  cv::Mat filtered;
  cv::medianBlur(mono8, filtered, 3);

  std::memcpy(out->data.data(), filtered.data, out->data.size());

  pub_->publish(std::move(out));

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000, "IR raw range: min=%u max=%u",
    min_value, max_value);
}

}  // namespace ir_converter

RCLCPP_COMPONENTS_REGISTER_NODE(ir_converter::IrY10Converter)
