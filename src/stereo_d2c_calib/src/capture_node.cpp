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

// D2C stereo capture: synchronized IR/color checkerboard pairs with
// per-side debug views. Depth and IR share the same sensor, so the
// IR<->color extrinsic doubles as the depth<->color (D2C) extrinsic.
#include "stereo_d2c_calib/capture_node.hpp"

#include <cstdio>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

namespace
{

using Image = sensor_msgs::msg::Image;

// Returns a read-only view for mono8 and one converted buffer for color input.
cv::Mat decodeGray(const Image::ConstSharedPtr & msg)
{
  if (msg->encoding == "mono8") {
    return cv::Mat(
      msg->height, msg->width, CV_8UC1,
      const_cast<uint8_t *>(msg->data.data()), msg->step);
  }
  if (msg->encoding != "rgb8" && msg->encoding != "bgr8") {
    throw std::runtime_error("unsupported encoding: " + msg->encoding);
  }
  const cv::Mat color(msg->height, msg->width, CV_8UC3,
    const_cast<uint8_t *>(msg->data.data()), msg->step);
  cv::Mat gray;
  cv::cvtColor(color, gray, msg->encoding == "rgb8" ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
  return gray;
}

cv::Mat toBgrDisplay(const cv::Mat & gray)
{
  cv::Mat bgr;
  cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}

}  // namespace

namespace stereo_d2c_calib
{

D2CCapture::D2CCapture(const rclcpp::NodeOptions & options)
: Node("d2c_capture", options)
{
  out_dir_ = declare_parameter<std::string>("out_dir", "/tmp/d2c_pairs");
  ir_topic_ = declare_parameter<std::string>("ir_topic", "/camera/ir/image_mono8");
  color_topic_ = declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
  cols_ = declare_parameter<int>("cols", 10);
  rows_ = declare_parameter<int>("rows", 7);
  square_ = declare_parameter<double>("square", 0.02);
  auto_interval_ = declare_parameter<double>("auto_interval", 2.0);
  if (auto_interval_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "auto_interval <= 0 with no manual mode; forcing 2.0 s");
    auto_interval_ = 2.0;
  }
  std::filesystem::create_directories(out_dir_);
  size_t next_index = 0;
  for (const auto & entry : std::filesystem::directory_iterator(out_dir_)) {
    const std::string filename = entry.path().filename().string();
    if (entry.path().extension() != ".yml" || filename.rfind("pair_", 0) != 0 ||
      filename.size() <= 9)
    {
      continue;
    }
    const std::string number = filename.substr(5, filename.size() - 5 - 4);
    if (number.empty() || number.find_first_not_of("0123456789") != std::string::npos) {
      RCLCPP_WARN(get_logger(), "ignoring malformed pair filename: %s", filename.c_str());
      continue;
    }
    try {
      const auto parsed = std::stoull(number);
      if (parsed >= std::numeric_limits<size_t>::max()) {
        RCLCPP_WARN(get_logger(), "ignoring exhausted pair filename: %s", filename.c_str());
        continue;
      }
      next_index = std::max(next_index, static_cast<size_t>(parsed + 1));
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "ignoring malformed pair filename: %s", filename.c_str());
    }
  }
  saved_ = next_index;

  debug_ir_pub_ = create_publisher<Image>("~/debug_ir", rclcpp::SensorDataQoS());
  debug_color_pub_ = create_publisher<Image>("~/debug_color", rclcpp::SensorDataQoS());

  // Both inputs are best-effort (v4l2 SensorDataQoS, ir_converter
  // sensor_data default); reliable subs would never match -> silence.
  sub_ir_.subscribe(this, ir_topic_, rmw_qos_profile_sensor_data);
  sub_color_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
  // Small queue: with ~30 Hz inputs and a heavy per-pair callback,
  // a deep queue only serves stale pairs (visible lag in debug views).
  sync_ = std::make_shared<Sync>(SyncPolicy(4), sub_ir_, sub_color_);
  sync_->registerCallback(&D2CCapture::callback, this);
  RCLCPP_INFO(
    get_logger(), "auto-capturing synchronized pairs in %s every %.1f s (Ctrl-C to stop)",
    out_dir_.c_str(), auto_interval_);
}

bool D2CCapture::findBoard(
  const cv::Mat & gray, int cols, int rows,
  std::vector<cv::Point2f> & corners)
{
  bool ok = cv::findChessboardCorners(
    gray, cv::Size(cols, rows), corners,
    cv::CALIB_CB_ADAPTIVE_THRESH |
    cv::CALIB_CB_NORMALIZE_IMAGE);
  if (ok) {
    cv::cornerSubPix(
      gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30,
        0.001));
  }
  return ok;
}

bool D2CCapture::diverse(
  const std::vector<cv::Point2f> & corners,
  const std::vector<cv::Point2f> & prev)
{
  if (prev.empty()) {
    return true;
  }
  double sum = 0.0;
  for (size_t i = 0; i < corners.size(); ++i) {
    sum += cv::norm(corners[i] - prev[i]);
  }
  return sum / corners.size() > 30.0;
}

void D2CCapture::publishSide(
  rclcpp::Publisher<Image>::SharedPtr pub, cv::Mat bgr,
  const std::vector<cv::Point2f> & corners, bool ok, const char * tag,
  size_t saved, const std_msgs::msg::Header & header)
{
  if (ok) {
    cv::drawChessboardCorners(bgr, cv::Size(cols_, rows_), corners, true);
  }
  cv::putText(
    bgr, std::string(tag) + (ok ? ":OK #" : ":-- #") + std::to_string(saved),
    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
  auto msg = std::make_unique<Image>();
  msg->header = header;
  msg->height = static_cast<uint32_t>(bgr.rows);
  msg->width = static_cast<uint32_t>(bgr.cols);
  msg->encoding = "bgr8";
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(bgr.cols * 3);
  msg->data.assign(bgr.data, bgr.data + bgr.total() * bgr.elemSize());
  pub->publish(std::move(msg));
}

void D2CCapture::callback(
  const Image::ConstSharedPtr & ir_msg,
  const Image::ConstSharedPtr & color_msg)
{
  cv::Mat ir_gray, color_gray;
  try {
    ir_gray = decodeGray(ir_msg);
    color_gray = decodeGray(color_msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    return;
  }
  std::vector<cv::Point2f> c_ir, c_color;
  const bool ok_ir = findBoard(ir_gray, cols_, rows_, c_ir);
  const bool ok_color = findBoard(color_gray, cols_, rows_, c_color);
  publishSide(
    debug_ir_pub_, toBgrDisplay(ir_gray), c_ir, ok_ir, "ir", saved_,
    ir_msg->header);
  // Debug drawing needs a writable BGR buffer; it is intentionally separate
  // from the grayscale image used for detection.
  cv::Mat color_bgr = toBgrDisplay(color_gray);
  publishSide(
    debug_color_pub_, color_bgr, c_color, ok_color, "color", saved_,
    color_msg->header);

  char status[128];
  snprintf(
    status, sizeof(status), "ir:%s color:%s saved:%zu AUTO/%.1fs (Ctrl-C to stop)",
    ok_ir ? "OK" : "--", ok_color ? "OK" : "--", saved_, auto_interval_);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s", status);
  const double now = this->now().seconds();
  if (ok_ir && ok_color && now - last_save_t_ >= auto_interval_) {
    if (diverse(c_ir, prev_ir_) && diverse(c_color, prev_color_)) {
      char path[512];
      snprintf(path, sizeof(path), "%s/pair_%03zu.yml", out_dir_.c_str(), saved_);
      cv::FileStorage fs(path, cv::FileStorage::WRITE);
      fs << "cols" << cols_ << "rows" << rows_ << "square" << square_;
      fs << "ir" << cv::Mat(c_ir).reshape(1);
      fs << "color" << cv::Mat(c_color).reshape(1);
      fs.release();
      prev_ir_ = c_ir;
      prev_color_ = c_color;
      last_save_t_ = now;
      RCLCPP_INFO(get_logger(), "saved pair %zu", saved_);
      ++saved_;
    }
  }
}

}  // namespace stereo_d2c_calib

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_d2c_calib::D2CCapture)
