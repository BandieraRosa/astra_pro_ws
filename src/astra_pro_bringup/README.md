# astra_pro_bringup

## Astra Pro bringup

Source the ROS and workspace environments in a fresh shell before launching:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch astra_pro_bringup astra_pro.launch.py
```

The Astra Pro depth/IR streams use the Orbbec SDK. Its SDK color stream is
disabled because this unit cannot start it; color comes from `/dev/video2`
through `v4l2_camera`. The UVC color stream is mirrored. `image_flip` publishes
the corrected stream on `/camera/color/image_flipped` and the corrected camera
info on `/camera/color/camera_info_flipped`.

For automatic D2C capture, use the single launch described in
`stereo_d2c_calib/README.md`. The default calibration capture intentionally
uses the raw mirrored color topic; the standalone solver must therefore receive
`--flip-color`.

The static depth-to-color TF in `astra_pro.launch.py` is an identity placeholder
until a real stereo calibration result has been validated.
