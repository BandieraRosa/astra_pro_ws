// D2C stereo solve: fixed-intrinsic stereoCalibrate over captured pairs.
// Usage:
//   d2c_solve --pairs DIR --ir-yaml IR.yaml --color-yaml COLOR.yaml
//             [--cols 10] [--rows 7] [--square 0.02] [--flip-color]
// With --flip-color, mirrored color corners/yaml are unmirrored in memory
// (x' = W - 1 - x, cx' = W - 1 - cx, p2' = -p2) so the solve runs true-domain.
// Prints R/T plus ready static_transform_publisher arguments.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <camera_calibration_parsers/parse.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace
{

struct Args {
  std::string pairs;
  std::string ir_yaml;
  std::string color_yaml;
  int cols = 10;
  int rows = 7;
  double square = 0.02;
  bool flip_color = false;
};

bool parseArgs(int argc, char **argv, Args &args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto need = [&](std::string &out) {
      if (i + 1 >= argc) {
        return false;
      }
      out = argv[++i];
      return true;
    };
    if (key == "--pairs") {
      if (!need(args.pairs)) {
        return false;
      }
    } else if (key == "--ir-yaml") {
      if (!need(args.ir_yaml)) {
        return false;
      }
    } else if (key == "--color-yaml") {
      if (!need(args.color_yaml)) {
        return false;
      }
    } else if (key == "--cols") {
      std::string v;
      if (!need(v)) {
        return false;
      }
      args.cols = std::stoi(v);
    } else if (key == "--rows") {
      std::string v;
      if (!need(v)) {
        return false;
      }
      args.rows = std::stoi(v);
    } else if (key == "--square") {
      std::string v;
      if (!need(v)) {
        return false;
      }
      args.square = std::stod(v);
    } else if (key == "--flip-color") {
      args.flip_color = true;
    } else if (key == "--help" || key == "-h") {
      return false;
    } else {
      std::cerr << "unknown arg: " << key << std::endl;
      return false;
    }
  }
  return !args.pairs.empty() && !args.ir_yaml.empty() && !args.color_yaml.empty();
}

bool loadCamInfo(const std::string &path, cv::Mat &k, cv::Mat &d, cv::Size &size)
{
  std::string name;
  sensor_msgs::msg::CameraInfo info;
  if (!camera_calibration_parsers::readCalibration(path, name, info)) {
    std::cerr << "failed to read " << path << std::endl;
    return false;
  }
  if (info.k.size() != 9 || info.d.size() < 4) {
    std::cerr << "bad K/D size in " << path << std::endl;
    return false;
  }
  k = cv::Mat(3, 3, CV_64F, const_cast<double *>(info.k.data())).clone();
  d = cv::Mat(1, static_cast<int>(info.d.size()), CV_64F,
              const_cast<double *>(info.d.data()))
          .clone();
  size = cv::Size(static_cast<int>(info.width), static_cast<int>(info.height));
  return true;
}

void rmatToQuat(const cv::Mat &r, double &qx, double &qy, double &qz, double &qw)
{
  const double trace = r.at<double>(0, 0) + r.at<double>(1, 1) + r.at<double>(2, 2);
  if (trace > 0.0) {
    const double s = 2.0 * std::sqrt(trace + 1.0);
    qw = 0.25 * s;
    qx = (r.at<double>(2, 1) - r.at<double>(1, 2)) / s;
    qy = (r.at<double>(0, 2) - r.at<double>(2, 0)) / s;
    qz = (r.at<double>(1, 0) - r.at<double>(0, 1)) / s;
  } else if (r.at<double>(0, 0) > r.at<double>(1, 1) &&
             r.at<double>(0, 0) > r.at<double>(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + r.at<double>(0, 0) - r.at<double>(1, 1) -
                                     r.at<double>(2, 2));
    qw = (r.at<double>(2, 1) - r.at<double>(1, 2)) / s;
    qx = 0.25 * s;
    qy = (r.at<double>(0, 1) + r.at<double>(1, 0)) / s;
    qz = (r.at<double>(0, 2) + r.at<double>(2, 0)) / s;
  } else if (r.at<double>(1, 1) > r.at<double>(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + r.at<double>(1, 1) - r.at<double>(0, 0) -
                                     r.at<double>(2, 2));
    qw = (r.at<double>(0, 2) - r.at<double>(2, 0)) / s;
    qx = (r.at<double>(0, 1) + r.at<double>(1, 0)) / s;
    qy = 0.25 * s;
    qz = (r.at<double>(1, 2) + r.at<double>(2, 1)) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + r.at<double>(2, 2) - r.at<double>(0, 0) -
                                     r.at<double>(1, 1));
    qw = (r.at<double>(1, 0) - r.at<double>(0, 1)) / s;
    qx = (r.at<double>(0, 2) + r.at<double>(2, 0)) / s;
    qy = (r.at<double>(1, 2) + r.at<double>(2, 1)) / s;
    qz = 0.25 * s;
  }
}

}  // namespace

int main(int argc, char **argv)
{
  Args args;
  if (!parseArgs(argc, argv, args)) {
    std::cerr << "usage: d2c_solve --pairs DIR --ir-yaml IR.yaml --color-yaml COLOR.yaml "
                 "[--cols 10] [--rows 7] [--square 0.02] [--flip-color]"
              << std::endl;
    return 1;
  }
  cv::Mat k_ir, d_ir, k_c, d_c;
  cv::Size size_ir, size_c;
  if (!loadCamInfo(args.ir_yaml, k_ir, d_ir, size_ir) ||
      !loadCamInfo(args.color_yaml, k_c, d_c, size_c)) {
    return 1;
  }
  if (size_ir != size_c) {
    std::cerr << "resolution mismatch" << std::endl;
    return 1;
  }
  double flip_w = 0.0;
  if (args.flip_color) {
    flip_w = static_cast<double>(size_c.width - 1);
    k_c.at<double>(0, 2) = flip_w - k_c.at<double>(0, 2);
    d_c.at<double>(0, 3) = -d_c.at<double>(0, 3);
    std::cout << "color unmirrored in memory (W=" << size_c.width << ")" << std::endl;
  }

  std::vector<std::vector<cv::Point3f>> objpoints;
  std::vector<std::vector<cv::Point2f>> img_ir, img_c;
  std::vector<cv::Point3f> objp;
  for (int r = 0; r < args.rows; ++r) {
    for (int c = 0; c < args.cols; ++c) {
      objp.emplace_back(c * args.square, r * args.square, 0.0f);
    }
  }
  for (const auto &entry : std::filesystem::directory_iterator(args.pairs)) {
    if (entry.path().extension() != ".yml") {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (name.rfind("pair_", 0) != 0) {
      continue;  // skip d2c_result.yml etc.
    }
    cv::FileStorage fs(entry.path().string(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "cannot open " << entry.path() << std::endl;
      continue;
    }
    cv::Mat mir, mc;
    fs["ir"] >> mir;
    fs["color"] >> mc;
    fs.release();
    if (mir.empty() || mc.empty()) {
      continue;
    }
    mir = mir.reshape(1);
    mc = mc.reshape(1);
    std::vector<cv::Point2f> vir, vc;
    vir.assign(reinterpret_cast<cv::Point2f *>(mir.data),
               reinterpret_cast<cv::Point2f *>(mir.data) + mir.rows);
    vc.assign(reinterpret_cast<cv::Point2f *>(mc.data),
              reinterpret_cast<cv::Point2f *>(mc.data) + mc.rows);
    if (flip_w > 0.0) {
      for (auto &p : vc) {
        p.x = static_cast<float>(flip_w) - p.x;
      }
    }
    if (vir.size() != static_cast<size_t>(args.cols * args.rows) ||
        vc.size() != static_cast<size_t>(args.cols * args.rows)) {
      std::cerr << "bad corner count in " << name << std::endl;
      continue;
    }
    objpoints.push_back(objp);
    img_ir.push_back(vir);
    img_c.push_back(vc);
  }
  if (objpoints.size() < 10) {
    std::cerr << "need >=10 pairs, got " << objpoints.size() << std::endl;
    return 1;
  }
  std::cout << "solving from " << objpoints.size() << " pairs ..." << std::endl;
  cv::Mat R, T, E, F;
  const double rmse = cv::stereoCalibrate(
      objpoints, img_ir, img_c, k_ir, d_ir, k_c, d_c, size_ir, R, T, E, F,
      cv::CALIB_FIX_INTRINSIC,
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 200, 1e-6));
  const double baseline = cv::norm(T);
  double qx, qy, qz, qw;
  rmatToQuat(R, qx, qy, qz, qw);
  printf("reprojection RMSE: %.4f px\n", rmse);
  printf("baseline |T|: %.4f m (expect ~0.02-0.04)\n", baseline);
  printf("T (m): %.6f %.6f %.6f\n", T.at<double>(0), T.at<double>(1), T.at<double>(2));
  printf("static_transform_publisher args:\n");
  printf("  %.6f %.6f %.6f %.6f %.6f %.6f %.6f camera_depth_optical_frame "
         "camera_color_optical_frame\n",
         T.at<double>(0), T.at<double>(1), T.at<double>(2), qx, qy, qz, qw);
  cv::FileStorage fs(args.pairs + "/d2c_result.yml", cv::FileStorage::WRITE);
  fs << "R" << R << "T" << T << "rmse" << rmse;
  fs.release();
  return 0;
}
