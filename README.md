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

- Test with Virtual Controller (L1 for reset, L2 for start/stop policy)
```bash
# Terminal 1
ros2 launch legged_control go2w.launch.py network_interface:=eth0 auto_stand:=true
```
```bash
# Terminal 2
./build/legged_control/tools/joystick_emu eth0
```

### Standalone run without ros:
check [standalone_entry](src/standalone)
```shell
./build/legged_control/aliengo_standalone
./build/legged_control/go2w_standalone
```

## Todo
- SDK2 go2 lidar/camera data integration