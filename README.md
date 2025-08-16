# VIKIT - ROS2版本

VIKIT (Vision-Kit) 提供了一些用于视觉/机器人项目的工具。
此版本已适配到ROS2。

## 主要变更

### 从ROS1到ROS2的适配变更：

1. **构建系统**：
   - 从 `catkin` 迁移到 `ament_cmake`
   - 更新了所有 `package.xml` 文件以使用ROS2格式
   - 更新了所有 `CMakeLists.txt` 文件

2. **依赖项**：
   - `roscpp` → `rclcpp`
   - `rospy` → `rclpy`
   - `tf` → `tf2_ros`
   - `cmake_modules` → `eigen3_cmake_module`

3. **API变更**：
   - `ros::Time` → `rclcpp::Time`
   - `ros::Duration` → `rclcpp::Duration`
   - `ros::Publisher` → `rclcpp::Publisher<T>::SharedPtr`
   - `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`
   - `visualization_msgs::Marker` → `visualization_msgs::msg::Marker`

4. **Python变更**：
   - `rosrun` → `ros2 run`
   - 参数传递格式从 `_param:=value` 改为 `--ros-args -p param:=value`

## 构建说明

```bash
# 在ROS2工作空间中
cd ~/ros2_ws/src
git clone <repository_url> rpg_vikit
cd ~/ros2_ws
colcon build --packages-select vikit_common vikit_ros vikit_py
```

## 依赖项

- ROS2 (Humble或更新版本)
- OpenCV
- Eigen3
- Sophus

## 注意事项

- 此版本需要ROS2环境
- 某些函数签名已更改以适应ROS2的API
- 参数访问现在需要节点上下文

## 许可证

- vikit_common: GPLv3
- vikit_ros: GPLv3  
- vikit_py: BSD

