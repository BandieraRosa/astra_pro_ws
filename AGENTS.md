# AGENTS.md — orbbec_ws (ROS 2 Humble, Ubuntu 22.04)

## Environment
- Always run ROS commands as: `source /opt/ros/humble/setup.bash && source install/setup.bash` (fresh shell each time; stale `AMENT_PREFIX_PATH` causes "package not found").
- Temp scratch: `/tmp/opencode`. Display exists (`DISPLAY=:0`).
- Build: `colcon build --packages-select <pkg>`. Install tree uses symlinks into `src/`, but **new files still need a build** to appear under `install/`.

## Packages (`src/`)
- `OrbbecSDK_ROS2` — git submodule (see below): `orbbec_camera` driver + msgs. We carry local driver patches.
- `astra_pro_bringup` — our launch + config: `astra_pro.launch.py` (production: orbbec depth/IR + v4l2 color in one container), `calibrate_d2c.launch.py` (adds calib container).
- `ir_converter` — mono16→mono8 (+median). `image_flip` — unmirror relay. `stereo_d2c_calib` — C++ D2C capture (`d2c_capture`) + solve (`d2c_solve`).

## Submodule rules
- Remote is the fork (`BandieraRosa/OrbbecSDK_ROS2`), branch pinned `main-legacy` (`.gitmodules` + `git submodule sync` after URL edits). Parent repo has **no remote**; only the submodule pushes.
- Driver patch workflow: edit in `src/OrbbecSDK_ROS2` → `colcon build` → commit **inside** submodule → push fork → `git add src/OrbbecSDK_ROS2` in parent.
- Our driver additions live in `ob_camera_node.{h,cpp}`: `overrideAstraProDepthCalibration`, `setupIrInfoManager`, `applyCalibratedCameraInfo`. (A factory depth→color-optical TF edge was tried and reverted: firmware D2C is all zeros. The identity edge lives in launch as `static_transform_publisher`.)

## Hardware truth (Astra Pro, PID 0x0403, FW RD107E-007, USB2.0)
- No 10 fps profiles exist (all @30, hi-res @7). SDK defaults (10 fps) make the driver `exit(-1)` and kill the whole container. Keep `*_fps: 30`.
- SDK color path is dead on this unit (profiles enumerate, pipeline start fails with `Match openni video mode failed`). Color **must** go via v4l2 (`/dev/video2` = Astra Pro UVC). Depth/IR via SDK.
- Factory D2C extrinsic is all zeros (placeholder). IR is Y10 (SDK unpacks to Y16); depth Y11. UVC color stream is **mirrored**; IR/depth are true-handed.

## Driver quirks (pinned `main-legacy`)
- `ir_info_url`/`color_info_url`/`enable_publish_extrinsic` are dead params upstream (read, never used). Our patch wires `ir_info_url` through `CameraInfoManager`.
- Any unmatched stream profile → `exit(-1)`, whole container dies. Check candidates with a tiny SDK probe (`pipeline.getStreamProfileList`) before trusting launch defaults.
- `publish_tf=true` + `tf_publish_rate>0` ⇒ TF only on `/tf`, `/tf_static` stays empty.
- IMU TF pair is published unconditionally with hardcoded names (no IMU on this device).

## Launch / YAML gotchas (launch_ros Humble)
- Param-file top keys must match **full node names** (`/camera/orbbec`), bare names are silently ignored; `/**` matches all.
- YAML files get **no type coercion**: quoted `"100"` into an int param crashes the node. Launch-arg strings **are** coerced.
- Only declared launch args are accepted from CLI. Later `parameters=[...]` entries override earlier ones (launch args beat yaml).
- `ComposableNode` needs a registered component plugin; plain executables must be `Node` actions.
- `extra_arguments=[{'use_intra_process_comms': True}]` enables intra-process; still requires QoS-compatible endpoints, and cross-container/bridge traffic always serializes.

## Python pins (do not upgrade)
- `numpy<2` (ROS binaries built against 1.x), `opencv-python==4.8.1.78` (v5 changed `findChessboardCorners` return shape and breaks `camera_calibration`).
- Humble QoS preset is `qos_profile_sensor_data` (no `SensorDataQoS` class until Jazzy).

## v4l2 specifics
- Publishes best-effort (`SensorDataQoS`); subscribers must not require reliable.
- `set_camera_info` resolves to namespace level (`/camera/color/set_camera_info`); the FQN variant in `ros2 service list` is stale daemon cache.
- Published `camera_info.header.frame_id` is empty (upstream only stamps); image header carries `camera_frame_id`.
- This camera exposes no HFLIP/VFLIP controls.

## Calibration files (`astra_pro_bringup/config/`)
- `astra_pro_ir.yaml` (plumb_bob, loaded into SDK structs at runtime), `astra_pro_color.yaml` (v4l2 `camera_info_url`), `astra_pro_params.yaml` (all node params, annotated).
- `CameraInfoManager` warns-but-loads on `camera_name` mismatch; keep names device-derived where noted in comments.
