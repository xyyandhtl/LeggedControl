# Unitree RL Locomotion policy C++ Deploy
RL Policy use ONNX Runtime。currently support Himloco policy.

## Usage
- Export policy
```bash
python scripts/export.py --jit path/to/model.jit
```
- SDK1（Aliengo）
```bash
ros2 launch legged_control sdk1_velocity.launch.py target_ip:=192.168.123.10
```
- SDK2（Go2/Go2W）
```bash
ros2 launch legged_control sdk2_velocity.launch.py network_interface:=eth0 auto_stand:=true
```

### Standalone run without ros:
check [standalone_entry](src/standalone)
```shell
./build/legged_control/aliengo_standalone
```

## Todo
SDK2 Joystick support
SDK2 go2 lidar/camera data integration