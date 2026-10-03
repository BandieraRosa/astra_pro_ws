# ir_converter

该包提供 `ir_converter::IrY10Converter` 组件，将 Orbbec 发布的 `mono16`
IR 图像转换为 `mono8`，支持：

- `linear`：按 Y10 的固定范围右移 2 位；
- `normalize`：按当前帧最小值和最大值做亮度拉伸；
- OpenCV 3x3 中值滤波。

组件输入和输出使用 SensorDataQoS。输出消息通过 `unique_ptr` 发布，以便
在同一 component container 内配合 intra-process communication，避免额外
的 ROS 消息复制。像素格式转换和滤波仍然需要独立的输出 buffer。

无硬件时可只构建该包：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select ir_converter
```
