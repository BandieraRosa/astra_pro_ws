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
#ifndef IMAGE_FLIP__CAMERA_INFO_FLIP_HPP_
#define IMAGE_FLIP__CAMERA_INFO_FLIP_HPP_

#include <sensor_msgs/msg/camera_info.hpp>

namespace image_flip
{

inline sensor_msgs::msg::CameraInfo horizontallyFlipped(
  sensor_msgs::msg::CameraInfo info)
{
  if (info.k.size() == 9) {
    info.k[2] = static_cast<double>(info.width - 1) - info.k[2];
  }
  if (info.p.size() == 12) {
    info.p[2] = static_cast<double>(info.width - 1) - info.p[2];
  }
  if (info.d.size() >= 4) {
    info.d[3] = -info.d[3];
  }
  return info;
}

}  // namespace image_flip

#endif  // IMAGE_FLIP__CAMERA_INFO_FLIP_HPP_
