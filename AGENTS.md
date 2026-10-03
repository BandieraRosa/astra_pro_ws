# AGENTS.md：orbbec_ws（ROS 2 Humble，Ubuntu 22.04）

## 环境与构建

- 每次执行 ROS 命令都使用新 shell，并按顺序执行：`source /opt/ros/humble/setup.bash && source install/setup.bash`。旧的 `AMENT_PREFIX_PATH` 会导致新包找不到。
- 临时文件放在 `/tmp/opencode`。当前环境有显示器，但无硬件时不要启动真实相机或假定存在 `/dev/video2`。
- 构建指定包：`colcon build --packages-select <包名>`。install 使用指向 src 的符号链接，但新文件仍必须重新构建后才会出现在 install。
- 计划和其他会话中间文件放在 `docs/plans/`，该目录已加入本机 Git 排除项，不纳入提交。

## 包与职责

- `src/OrbbecSDK_ROS2`：固定 `main-legacy` 的 git 子模块，包含 Orbbec 驱动、消息和描述包；仓库中有本地驱动补丁。
- `src/astra_pro_bringup`：Astra Pro 参数、标定文件和 launch。生产 launch 启动 Orbbec 深度/IR、V4L2 color 和翻转组件；标定 launch 将相机和标定组件放入同一个 container。
- `src/ir_converter`：Y10/mono16 到 mono8 的转换、亮度归一化和 3x3 中值滤波。
- `src/image_flip`：水平翻转图像及 CameraInfo，既提供 standalone executable，也注册 composable component。
- `src/stereo_d2c_calib`：C++ D2C 采集组件 `d2c_capture` 和离线解算器 `d2c_solve`。

## 子模块规则

- 远程是 `BandieraRosa/OrbbecSDK_ROS2` fork，分支固定为 `main-legacy`；修改 URL 后要执行 `git submodule sync`。
- 驱动修改流程：编辑 `src/OrbbecSDK_ROS2`，构建验证，在子模块内提交并推送 fork，最后在父仓库提交 gitlink。
- 自定义驱动逻辑位于 `ob_camera_node.{h,cpp}`，包括 IR CameraInfoManager 加载和 Astra Pro 深度标定覆盖。

## Astra Pro 硬件事实

- 设备是 PID `0x0403`、固件 `RD107E-007`、USB2.0 的 Astra Pro。
- 本设备没有可用的 10 fps profile；深度、IR 等参数保持 `*_fps: 30`，否则驱动可能直接退出并带死整个 container。
- SDK color 流无法启动，必须使用 `/dev/video2` 的 V4L2 color；深度和 IR 仍走 SDK。
- IR 是 Y10，SDK 发布为 Y16；深度是 Y11；UVC color 原始图像水平镜像，IR/深度不是镜像。
- 固件内置 D2C 外参是全零占位，launch 中的 identity TF 只是待标定占位，不能作为真实外参使用。

## ROS launch 与参数陷阱

- YAML 参数顶层键必须是完整节点名，例如 `/camera/orbbec`；裸节点名会被静默忽略。
- YAML 不做类型转换，整数和布尔值不要写成带引号的字符串。
- launch 命令行只能传已声明的参数；后写入的 `parameters` 会覆盖前面的参数。
- `ComposableNode` 必须有已注册的组件插件；普通 executable 才使用 `Node`。
- `extra_arguments=[{'use_intra_process_comms': True}]` 只在同一进程内启用 intra-process。跨 container 的 DDS 和 bridge/Foxglove 输出仍然必须序列化，不能称为端到端零拷贝。

## 零拷贝约束

- 完整标定 launch 将 Orbbec、V4L2、image flip、IR converter 和 D2C capture 放在同一个 component container，并为组件启用 intra-process。
- ROS 图像 publisher 使用 `std::unique_ptr` 发布，组件间可避免不必要的消息复制；图像格式转换、中值滤波、翻转和 debug 绘制本身仍需要新像素 buffer。
- `with_cameras:=false` 会让标定组件连接外部相机进程，此时只有标定 container 内部可能 intra-process。
- bridge、Foxglove、跨进程录包和网络传输是明确的序列化边界；无硬件环境只能做代码级和合成数据验证，不能证明运行时内存地址复用。

## QoS 与设备专用规则

- V4L2 图像使用 best-effort SensorDataQoS，订阅者必须使用兼容的 sensor-data QoS。
- debug 图像也使用 SensorDataQoS，避免可靠传输积压旧帧；不以降低发布频率作为零拷贝替代方案。
- `publish_tf=true` 且 `tf_publish_rate>0` 时，TF 只会持续发布到 `/tf`；需要静态 TF 时使用 `static_transform_publisher`。
- `camera_name` 改名时必须同步检查 YAML 完整节点名、话题、frame_id 和静态 TF。

## 标定操作

- 无硬件时不要执行真实 launch；可执行构建、Python launch 语法检查、离线 `d2c_solve` 和合成数据测试。
- 有硬件时启动：`ros2 launch astra_pro_bringup calibrate_d2c.launch.py pairs_dir:=/tmp/d2c_pairs`。
- 默认采集 raw 镜像 color，离线解算必须加 `--flip-color`；如果改用已翻正话题，不能再次加该选项。
- 棋盘格内角点默认 `10x7`，对应 `11x8` 方格，方格边长 `0.02 m`。
