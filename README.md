# Unitree RL Locomotion policy C++ Deploy
RL Policy use ONNX runtime, currently support Himloco policy.

## Usage
- Export policy
```bash
python scripts/export.py --jit path/to/policy.pt
```
- Build
```bash
# if ENABLE_ORT_CUDA, onnxruntime is v1.16.0 which support cuda 11.x, compatible with jetpack 5.1
# if need ENABLE_ORT_CUDA on cuda 12.x, replace onnxruntime with higher version, or just test with cpu
colcon build --packages-select legged_control --cmake-clean-cache --symlink-install
```

- SDK1（Aliengo）
```bash
ros2 launch legged_control aliengo.launch.py target_ip:=192.168.123.10
```
- SDK2（Go2/Go2W）
```text
if build on ros2 foxy, need to Compile cyclone-dds and set envs as [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2/tree/master?tab=readme-ov-file#1-network-configuration)
```
```bash
# this node should not set RMW_IMPLEMENTATION/CYCLONEDDS_URI, otherwise would not successfully run (don't know why)
ros2 launch legged_control go2w.launch.py network_interface:=enx0826ae330a17 [control_mode:=policy]
```
add `control_mode:=policy` to take Policy Mode.

- Keyboard Teleop Simulator

This node allows you to simulate both a DDS-based joystick and a ROS 2 /cmd_vel publisher using your keyboard.
Note:

| DDS (Simulated Joystick)    | ROS (/cmd_vel) Commands     |
|-----------------------------|-----------------------------|
| w / s: Forward / Backward   | i / k: Forward / Backward   |
| a / d: Strafe Left / Right  | j / l: Strafe Left / Right  |
| q / e: Turn Left / Right    | u / o: Turn Left / Right    |

 - Button Commands (DDS) Note: These only affect nodes running in Policy Mode.
   - 2: Press L2 (Activate Policy / Toggle Run-Pause)
   - 1: Press L1 (Safe Exit: Stop policy and reset joints)
   - Ctrl+C: Quit the simulator.

```bash
# Terminal 1
ros2 launch legged_control go2w.launch.py network_interface:=enx0826ae330a17
```
```bash
# Terminal 2
ros2 run legged_control joystick_simu_node --ros-args -p network_interface:=enx0826ae330a17
```

### Standalone run without ros:
check [standalone_entry](src/standalone) (dont completed)
```shell
# Terminal 1
./build/legged_control/standalone/aliengo_standalone
./build/legged_control/standalone/go2w_standalone
```

