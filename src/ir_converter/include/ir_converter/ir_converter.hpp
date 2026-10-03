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

#ifndef IR_CONVERTER__IR_CONVERTER_HPP_
#define IR_CONVERTER__IR_CONVERTER_HPP_

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ir_converter
{

class IrY10Converter : public rclcpp::Node
{
public:
  explicit IrY10Converter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  static rclcpp::QoS makeQoS(const std::string & profile, int depth);
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string mode_;
};

}  // namespace ir_converter

#endif  // IR_CONVERTER__IR_CONVERTER_HPP_
