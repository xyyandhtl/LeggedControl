### 项目简介

基于 ROS 2 的 Unitree 机器人控制包（`unitree_control`）。提供：
- **SDK2 速度控制节点**：订阅 `/cmd_vel`（`geometry_msgs/Twist`），通过 Unitree SDK2 的 `SportClient` 控制 Go2/Go2W 机器人。
- **SDK1 速度控制节点**：订阅 `/cmd_vel`（`geometry_msgs/Twist`），通过 Unitree SDK1 的 UDP 通信控制 Go1/Aliengo 机器人。
- 示例程序：包含 Unitree SDK 直连 DDS 的低层示例（`example/go2`, `example/go2w`）与一个 LibTorch 推理演示（`example/main.cpp`）。

### 目录结构

- `src/`：ROS 2 节点源码
  - `sdk2_velocity_node.cpp`：SDK2 速度控制节点（Go2/Go2W）
  - `sdk1_velocity_node.cpp`：SDK1 速度控制节点（Go1/Aliengo）
- `launch/`：启动文件
  - `velocity.launch.py`：SDK2 速度控制节点启动文件
  - `sdk1_velocity.launch.py`：SDK1 速度控制节点启动文件
- `include/`：Unitree SDK 与 CycloneDDS 相关头文件。
  - `unitree_sdk1/`：老版本 SDK（UDP 通信，用于 Go1/Aliengo）
  - `unitree_sdk2/`：新版本 SDK（DDS 通信，用于 Go2/Go2W）
- `lib/`：依赖库（`x86_64/`、`aarch64/`）。
- `example/`：示例程序（可通过总 CMakeLists.txt 构建）
  - `aliengo/`：Go1/Aliengo 示例（使用 unitree_sdk1）
  - `go2/`：Go2 示例（使用 unitree_sdk2）
  - `go2w/`：Go2W 示例（使用 unitree_sdk2）
- `CMakeLists.txt`、`package.xml`：ROS 2 ament 构建配置与包信息。

### 依赖

- ROS 2：`ament_cmake`, `rclcpp`, `geometry_msgs`, `std_msgs`, `sensor_msgs`
- Unitree SDK 2：`libunitree_sdk2.a` 及对应头文件（用于 Go2/Go2W）
- Unitree SDK 1：`libunitree_sdk1.a` 及对应头文件（用于 Go1/Aliengo）
- CycloneDDS C/C++：已内置 `libddscxx.so`, `libddsc.so` 与头文件
- 可选：LibTorch（仅用于 `example/main.cpp`）

### 构建

#### ROS 2 工作空间构建（推荐）
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/lenovo/Projects/Embodied
colcon build --symlink-install --packages-select unitree_control
source install/setup.bash
```

#### 独立构建示例程序
```bash
cd /home/lenovo/Projects/Embodied/src/RobotControl
./build_examples.sh        # 构建示例
./build_examples.sh clean  # 清理构建
```

#### 安装后的使用
构建完成后，所有示例程序都会安装到 `install/unitree_control/lib/unitree_control/examples/` 目录：

```bash
# 运行安装的示例程序
# Go2 示例
install/unitree_control/lib/unitree_control/examples/go2_low_level enp3s0

# Aliengo 示例
install/unitree_control/lib/unitree_control/examples/aliengo_velocity

# 或者通过 ROS2 运行（如果配置了环境）
source install/setup.bash
ros2 run unitree_control go2_low_level enp3s0
```

> 注意：根据架构自动使用 `lib/x86_64` 或 `lib/aarch64`。确保其中含有：
> - `libunitree_sdk2.a`、`libddscxx.so`、`libddsc.so`（用于 Go2/Go2W）
> - `libunitree_sdk1.a`（用于 Go1/Aliengo）

### 运行

#### SDK2 速度控制节点（Go2/Go2W）
- 启动文件：
```bash
ros2 launch unitree_control velocity.launch.py \
  network_interface:=eth0 control_rate_hz:=50 stale_timeout_s:=0.5 \
  auto_stand:=true max_vx:=1.5 max_vy:=0.5 max_wz:=1.5
```

- 直接运行：
```bash
ros2 run unitree_control velocity_node --ros-args -p network_interface:=eth0
```

#### SDK1 速度控制节点（Go1/Aliengo）
- 启动文件：
```bash
ros2 launch unitree_control sdk1_velocity.launch.py \
  target_ip:=192.168.123.10 target_port:=8007 local_port:=8082 \
  control_rate_hz:=500 stale_timeout_s:=0.5 \
  max_vx:=2.0 max_vy:=1.0 max_wz:=2.0
```

- 直接运行：
```bash
ros2 run unitree_control sdk1_velocity_node --ros-args -p target_ip:=192.168.123.10
```

### 节点参数（默认值）

#### SDK2 速度控制节点（Go2/Go2W）
- `network_interface`: `"eth0"`
- `control_rate_hz`: `50`
- `stale_timeout_s`: `0.5`
- `auto_stand`: `true`
- `max_vx`: `1.5`, `max_vy`: `0.5`, `max_wz`: `1.5`

> 行为：收到 `/cmd_vel` 后以固定频率调用 `Move(vx, vy, wz)`；若超过 `stale_timeout_s` 未收到新指令，发送一次 `StopMove` 并清零。

#### SDK1 速度控制节点（Go1/Aliengo）
- `target_ip`: `"192.168.123.10"`
- `target_port`: `8007`
- `local_port`: `8082`
- `control_rate_hz`: `500`
- `stale_timeout_s`: `0.5`
- `max_vx`: `2.0`, `max_vy`: `1.0`, `max_wz`: `2.0`

> 行为：基于 `example_velocity_aliengo.cpp`，通过 UDP 通信进行低层控制。收到 Twist 后应用重力补偿和速度控制；若超过 `stale_timeout_s` 未收到新指令，停止所有电机运动。

### 示例（可通过总 CMakeLists.txt 构建）

- **Aliengo/Go1 示例**（使用 unitree_sdk1，UDP 通信）：
  - `aliengo_position`：位置控制示例
  - `aliengo_velocity`：速度控制示例  
  - `aliengo_walk`：行走控制示例
  - `aliengo_joystick`：手柄控制示例
  - `aliengo_torque`：力矩控制示例

- **Go2 示例**（使用 unitree_sdk2，DDS 通信）：
  - `go2_low_level`：低层控制示例
  - `go2_sport_client`：运动控制客户端
  - `go2_stand_example`：站立示例
  - `go2_robot_state_client`：机器人状态客户端
  - `go2_video_client`：视频客户端
  - `go2_vui_client`：语音交互客户端

- **Go2W 示例**（使用 unitree_sdk2，DDS 通信）：
  - `go2w_sport_client`：运动控制客户端
  - `go2w_stand_example`：站立示例

- **独立构建**（如需要）：
```bash
# 构建所有示例
cd /home/lenovo/Projects/Embodied
colcon build --symlink-install --packages-select unitree_control

# 或单独构建示例
cd /home/lenovo/Projects/Embodied/src/RobotControl/example/go2
mkdir -p build && cd build && cmake .. && make -j
./go2_low_level enp3s0
```

- 推理演示：`example/main.cpp`（需 LibTorch 与模型文件）。

### 常见问题

- 找不到 `.so`：已为 `velocity_node` 设置 rpath 指向 `lib/<arch>`；若仍失败，检查库是否匹配架构或设置 `LD_LIBRARY_PATH`。
- 无法连接：确认 `network_interface` 为实际网卡且与机器人同网段，并放行相关 UDP。
- 安全性：初次实验请降低 `max_*`，并确保机器人处于安全姿态。

### 许可

MIT（见 `package.xml`）。
