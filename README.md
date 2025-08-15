# Unitree RL Locomotion policy C++ Deploy

因目前开源部署流程大多为 Pytorch 脚本，不方便 ROS2 集成部署分层导航框架，开发基于官方C++ SDK的 Unitree 机器狗的 ROS2 控制包（Go1/Aliengo 使用 SDK1 UDP；Go2/Go2W 使用 SDK2 DDS），方便以后只改底层实现的其他机器人部署。RL Policy 使用 ONNX Runtime 推理。目前样例仅支持 Himloco Policy 自用，暂不考虑扩展为多policy适配。

## 功能
- SDK1 速度控制节点：订阅 /cmd_vel，通过 UDP 控制 Go1/Aliengo，并使用 ONNX Runtime 计算 12 关节位置。
- SDK2 速度控制节点：订阅 /cmd_vel，通过 SportClient 控制 Go2/Go2W，待加入RL Policy运控。

## 依赖
- ROS 2: ament_cmake, rclcpp, geometry_msgs, std_msgs, sensor_msgs
- include/ lib/为三方依赖库，实际应放在third_party/，只为减少一层目录，现包括：
  - Unitree SDK1、SDK2 与其依赖 CycloneDDS
  - ONNX Runtime CPU版（GPU版简单修改即可）

## 运行
- SDK1（Go1/Aliengo）
```bash
ros2 launch unitree_control sdk1_velocity.launch.py target_ip:=192.168.123.10
```
- SDK2（Go2/Go2W）
```bash
ros2 launch unitree_control sdk2_velocity.launch.py network_interface:=eth0 auto_stand:=true
```

## SDK1 推理与配置
- 类名：SDK1RobotControl
- 观测配置结构：SDK1RobotObsConfig（含 onnx_model_path、obs_size、history_steps 等）
- 模型输入：shape [1, obs_size * history_steps]（整段历史拼接），输出：1x12（关节位置）
- 配置存放位置：$HOME/.config/legged/sdk1_config.ini
