# image_flip

该包将水平镜像的图像恢复到正常坐标系，同时修正 CameraInfo：

- 像素坐标使用 `x' = width - 1 - x`；
- 主点使用 `cx' = width - 1 - cx`；
- OpenCV 畸变数组中的 `p2` 符号取反。

包同时提供：

- standalone executable：`flip_node`；
- composable component：`ImageFlip`。

标定完整 launch 使用 component 版本，使 V4L2、翻转节点、IR converter
和 D2C capture 可以处于同一 container。输出图像仍需一次翻转像素复制，
这是算法本身的必要工作，不属于 ROS 消息额外复制。
