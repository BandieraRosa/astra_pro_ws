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
#include <gtest/gtest.h>

#include "image_flip/camera_info_flip.hpp"

TEST(CameraInfoFlip, ReflectsPixelCenterCoordinates)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 480;
  info.k = {500.0, 0.0, 319.25, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  info.p = {500.0, 0.0, 319.25, 0.0, 0.0, 500.0, 240.0, 0.0,
    0.0, 0.0, 1.0, 0.0};
  info.d = {0.1, 0.2, 0.01, -0.03, 0.0};

  const auto flipped = image_flip::horizontallyFlipped(info);

  EXPECT_DOUBLE_EQ(flipped.k[2], 319.75);
  EXPECT_DOUBLE_EQ(flipped.p[2], 319.75);
  EXPECT_DOUBLE_EQ(flipped.d[2], info.d[2]);
  EXPECT_DOUBLE_EQ(flipped.d[3], 0.03);
}
