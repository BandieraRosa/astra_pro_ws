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
  explicit IrY10Converter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  static rclcpp::QoS makeQoS(const std::string &profile, int depth);
  void callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string mode_;
};

}  // namespace ir_converter

#endif  // IR_CONVERTER__IR_CONVERTER_HPP_
