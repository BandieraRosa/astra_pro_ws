#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class IrY10Converter : public rclcpp::Node
{
 public:
  explicit IrY10Converter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("ir_converter", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/ir/image_raw");
    output_topic_ =
        declare_parameter<std::string>("output_topic", "/camera/ir/image_mono8");
    const auto qos_profile = declare_parameter<std::string>("qos_profile", "sensor_data");
    const auto qos_depth = declare_parameter<int>("qos_depth", 10);

    const auto qos = makeQoS(qos_profile, qos_depth);

    sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_, qos,
        std::bind(&IrY10Converter::callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, qos);

    RCLCPP_INFO(get_logger(),
                "Y10/mono16 -> mono8 converter started: %s -> %s (qos=%s, depth=%d)",
                input_topic_.c_str(), output_topic_.c_str(), qos_profile.c_str(),
                static_cast<int>(qos_depth));
  }

 private:
  static rclcpp::QoS makeQoS(const std::string& profile, int depth)
  {
    const int history_depth = depth > 0 ? depth : 10;
    if (profile == "default")
    {
      return rclcpp::QoS(history_depth);
    }
    if (profile == "system_default")
    {
      return rclcpp::SystemDefaultsQoS();
    }
    return rclcpp::SensorDataQoS();
  }
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->encoding != "mono16")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Expected mono16, got: %s",
                           msg->encoding.c_str());
      return;
    }

    sensor_msgs::msg::Image out;

    out.header = msg->header;
    out.height = msg->height;
    out.width = msg->width;
    out.encoding = "mono8";
    out.is_bigendian = false;
    out.step = msg->width;
    out.data.resize(static_cast<size_t>(out.height) * out.step);

    uint16_t min_value = std::numeric_limits<uint16_t>::max();

    uint16_t max_value = 0;

    for (uint32_t y = 0; y < msg->height; ++y)
    {
      const uint8_t* row = msg->data.data() + static_cast<size_t>(y) * msg->step;

      for (uint32_t x = 0; x < msg->width; ++x)
      {
        uint16_t value;

        std::memcpy(&value, row + x * sizeof(uint16_t), sizeof(uint16_t));

        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);

        // Astra Pro Y10:
        // 10-bit -> 8-bit
        uint16_t scaled = value >> 2;

        if (scaled > 255) scaled = 255;

        out.data[static_cast<size_t>(y) * out.step + x] = static_cast<uint8_t>(scaled);
      }
    }

    cv::Mat mono8(out.height, out.width, CV_8UC1, out.data.data(), out.step);

    cv::Mat filtered;
    cv::medianBlur(mono8, filtered, 3);

    std::memcpy(out.data.data(), filtered.data, out.data.size());

    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "IR raw range: min=%u max=%u",
                         min_value, max_value);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<IrY10Converter>());

  rclcpp::shutdown();

  return 0;
}