# stereo_d2c_calib

The capture component automatically saves synchronized IR/color checkerboard
pairs. It is headless: move the 11x8-square board between poses and stop with
Ctrl-C after at least 15--20 varied pairs have been saved.

Start the complete camera and calibration stack in one terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch astra_pro_bringup calibrate_d2c.launch.py pairs_dir:=/tmp/d2c_pairs
```

The default capture color topic is `/camera/color/image_raw`. Astra Pro's UVC
color image is mirrored, so solve captured pairs with `--flip-color`:

```bash
ros2 run stereo_d2c_calib d2c_solve \
  --pairs /tmp/d2c_pairs \
  --ir-yaml src/astra_pro_bringup/config/astra_pro_ir.yaml \
  --color-yaml src/astra_pro_bringup/config/astra_pro_color.yaml \
  --flip-color
```

The solver prints the calibrated transform from
`camera_depth_optical_frame` to `camera_color_optical_frame`. Do not replace
the identity TF in the bringup launch until the RMSE and baseline are sane.

To use cameras that are already running, pass `with_cameras:=false`. To adjust
the automatic save period, pass `auto_interval:=<seconds>`.
