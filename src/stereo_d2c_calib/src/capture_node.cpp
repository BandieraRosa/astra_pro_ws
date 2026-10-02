// D2C stereo capture: synchronized IR/color checkerboard pairs with
// per-side debug views. Depth and IR share the same sensor, so the
// IR<->color extrinsic doubles as the depth<->color (D2C) extrinsic.
#include "stereo_d2c_calib/capture_node.hpp"

#include <cstdio>
#include <filesystem>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

namespace
{

using Image = sensor_msgs::msg::Image;

// Returns grayscale working copy; throws on unsupported encoding.
cv::Mat decodeGray(const Image::ConstSharedPtr &msg)
{
  cv::Mat view;
  if (msg->encoding == "mono8") {
    view = cv::Mat(msg->height, msg->width, CV_8UC1,
                   const_cast<uint8_t *>(msg->data.data()), msg->step);
  } else if (msg->encoding == "rgb8") {
    view = cv::Mat(msg->height, msg->width, CV_8UC3,
                   const_cast<uint8_t *>(msg->data.data()), msg->step);
    cv::cvtColor(view, view, cv::COLOR_RGB2GRAY);
  } else if (msg->encoding == "bgr8") {
    view = cv::Mat(msg->height, msg->width, CV_8UC3,
                   const_cast<uint8_t *>(msg->data.data()), msg->step);
    cv::cvtColor(view, view, cv::COLOR_BGR2GRAY);
  } else {
    throw std::runtime_error("unsupported encoding: " + msg->encoding);
  }
  return view.clone();
}

cv::Mat toBgrDisplay(const cv::Mat &gray)
{
  cv::Mat bgr;
  cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}

}  // namespace

namespace stereo_d2c_calib
{

D2CCapture::D2CCapture(const rclcpp::NodeOptions &options) : Node("d2c_capture", options)
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
  size_t existing = 0;
  for (const auto &entry : std::filesystem::directory_iterator(out_dir_)) {
    if (entry.path().extension() == ".yml" &&
        entry.path().filename().string().rfind("pair_", 0) == 0) {
      ++existing;
    }
  }
  saved_ = existing;

  debug_ir_pub_ = create_publisher<Image>("~/debug_ir", 10);
  debug_color_pub_ = create_publisher<Image>("~/debug_color", 10);

  // Both inputs are best-effort (v4l2 SensorDataQoS, ir_converter
  // sensor_data default); reliable subs would never match -> silence.
  sub_ir_.subscribe(this, ir_topic_, rmw_qos_profile_sensor_data);
  sub_color_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
  sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_ir_, sub_color_);
  sync_->registerCallback(&D2CCapture::callback, this);
  RCLCPP_INFO(get_logger(), "auto-capturing synchronized pairs in %s every %.1f s (Ctrl-C to stop)",
              out_dir_.c_str(), auto_interval_);
}

bool D2CCapture::findBoard(const cv::Mat &gray, int cols, int rows,
                           std::vector<cv::Point2f> &corners)
{
  bool ok = cv::findChessboardCorners(gray, cv::Size(cols, rows), corners,
                                      cv::CALIB_CB_ADAPTIVE_THRESH |
                                          cv::CALIB_CB_NORMALIZE_IMAGE);
  if (ok) {
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30,
                                      0.001));
  }
  return ok;
}

bool D2CCapture::diverse(const std::vector<cv::Point2f> &corners,
                         const std::vector<cv::Point2f> &prev)
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

void D2CCapture::publishSide(rclcpp::Publisher<Image>::SharedPtr pub, cv::Mat bgr,
                             const std::vector<cv::Point2f> &corners, bool ok, const char *tag,
                             size_t saved, const std_msgs::msg::Header &header)
{
  if (ok) {
    cv::drawChessboardCorners(bgr, cv::Size(cols_, rows_), corners, true);
  }
  cv::putText(bgr, std::string(tag) + (ok ? ":OK #" : ":-- #") + std::to_string(saved),
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

void D2CCapture::callback(const Image::ConstSharedPtr &ir_msg,
                          const Image::ConstSharedPtr &color_msg)
{
  cv::Mat ir_gray, color_gray;
  try {
    ir_gray = decodeGray(ir_msg);
    color_gray = decodeGray(color_msg);
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    return;
  }
  std::vector<cv::Point2f> c_ir, c_color;
  const bool ok_ir = findBoard(ir_gray, cols_, rows_, c_ir);
  const bool ok_color = findBoard(color_gray, cols_, rows_, c_color);
  publishSide(debug_ir_pub_, toBgrDisplay(ir_gray), c_ir, ok_ir, "ir", saved_,
              ir_msg->header);
  // color_msg may be rgb8: rebuild a BGR display copy for drawing
  cv::Mat color_bgr;
  if (color_msg->encoding == "rgb8") {
    cv::Mat tmp(color_msg->height, color_msg->width, CV_8UC3,
                const_cast<uint8_t *>(color_msg->data.data()), color_msg->step);
    cv::cvtColor(tmp, color_bgr, cv::COLOR_RGB2BGR);
  } else {
    color_bgr = toBgrDisplay(color_gray);
  }
  publishSide(debug_color_pub_, color_bgr, c_color, ok_color, "color", saved_,
              color_msg->header);

  cv::Mat vis_ir, vis_color;
  cv::resize(toBgrDisplay(ir_gray), vis_ir, cv::Size(640, 480));
  cv::resize(color_bgr, vis_color, cv::Size(640, 480));
  cv::Mat vis;
  cv::hconcat(std::vector<cv::Mat>{vis_ir, vis_color}, vis);
  char status[128];
  snprintf(status, sizeof(status), "ir:%s color:%s saved:%zu AUTO/%.1fs (Ctrl-C to stop)",
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
