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
```bash
ros2 launch legged_control go2w.launch.py network_interface:=eth0 auto_stand:=true
```

- Test with Joystick Controller (Virtual)
```bash
# Terminal 1
ros2 launch legged_control go2w.launch.py network_interface:=eth0 auto_stand:=true
```
```bash
# Terminal 2
./build/legged_control/tools/joystick_emu eth0
```

1.  **L2 Button (Enter and Control Policy):**
    * **Default State:** The system starts in High-Level Sport Mode (`SPORT_MODE`).
    * **First L2 Press:** Switches from `SPORT_MODE` to Low-Level Policy Mode (`POLICY_MODE`). Upon switching, the policy is in a **paused** state, awaiting further commands.
    * **Subsequent L2 Presses (in Policy Mode):** Toggles the policy between "Running" and "Paused" states.

2.  **L1 Button (Safe Exit and Reset):**
    * Pressing L1 at any time triggers a safety procedure: it **pauses** the policy, switches back to `SPORT_MODE`, and commands the robot to perform the **default pose**.

### Standalone run without ros:
check [standalone_entry](src/standalone)
```shell
# Terminal 1
./build/legged_control/standalone/aliengo_standalone
./build/legged_control/standalone/go2w_standalone
```
```shell
# Terminal 2
./build/legged_control/tools/joystick_emu
```

## Todo
- SDK2 go2 lidar/camera data integration