cmake_minimum_required(VERSION 3.10)
project(legged_control)

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};/home/lenovo/Softwares/libtorch/)

# 选项
option(BUILD_EXAMPLES "Build example programs" ON)
# ONNX Runtime EP 选择
option(ENABLE_ORT_CUDA "Enable ONNX Runtime CUDA Execution Provider" OFF)
option(ENABLE_ORT_TENSORRT "Enable ONNX Runtime TensorRT Execution Provider" OFF)

# SDK路径
set(UNITREE_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

# 包含目录
include_directories(
  ${UNITREE_SDK_PATH}/include
  ${UNITREE_SDK_PATH}/include/unitree_sdk2
  ${UNITREE_SDK_PATH}/include/unitree_sdk1
  ${UNITREE_SDK_PATH}/include/ddscxx
  ${UNITREE_SDK_PATH}/include/onnxruntime
)

# 架构选择库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(UNITREE_LIB_PATH ${UNITREE_SDK_PATH}/lib/aarch64)
else()
  set(UNITREE_LIB_PATH ${UNITREE_SDK_PATH}/lib/x86_64)
endif()
link_directories(${UNITREE_LIB_PATH})

# SDK库别名
add_library(unitree_sdk1 INTERFACE)
target_include_directories(unitree_sdk1 INTERFACE
  ${UNITREE_SDK_PATH}/include
  ${UNITREE_SDK_PATH}/include/unitree_sdk1
)
target_link_libraries(unitree_sdk1 INTERFACE
  ${UNITREE_LIB_PATH}/libunitree_legged_sdk.so
  ${UNITREE_LIB_PATH}/libonnxruntime.so
)

add_library(unitree_sdk2 INTERFACE)
target_include_directories(unitree_sdk2 INTERFACE
  ${UNITREE_SDK_PATH}/include
  ${UNITREE_SDK_PATH}/include/unitree_sdk2
  ${UNITREE_SDK_PATH}/include/ddscxx
)
target_link_libraries(unitree_sdk2 INTERFACE
  ${UNITREE_LIB_PATH}/libunitree_sdk2.a
  ${UNITREE_LIB_PATH}/libddscxx.so
  ${UNITREE_LIB_PATH}/libddsc.so
  ${UNITREE_LIB_PATH}/libonnxruntime.so
  pthread
  dl
)

# 按需启用 ORT 的 CUDA/TensorRT Provider 并定义编译宏
set(ORT_PROVIDER_LIBS "")
if(ENABLE_ORT_CUDA)
  list(APPEND ORT_PROVIDER_LIBS ${UNITREE_LIB_PATH}/libonnxruntime_providers_cuda.so)
endif()
if(ENABLE_ORT_TENSORRT)
  list(APPEND ORT_PROVIDER_LIBS ${UNITREE_LIB_PATH}/libonnxruntime_providers_tensorrt.so)
endif()
if(ENABLE_ORT_CUDA OR ENABLE_ORT_TENSORRT)
  list(APPEND ORT_PROVIDER_LIBS ${UNITREE_LIB_PATH}/libonnxruntime_providers_shared.so)
  list(REMOVE_DUPLICATES ORT_PROVIDER_LIBS)
  # 链接到两个 SDK 接口库，使下游可执行自动继承
  target_link_libraries(unitree_sdk1 INTERFACE ${ORT_PROVIDER_LIBS})
  target_link_libraries(unitree_sdk2 INTERFACE ${ORT_PROVIDER_LIBS})
endif()

# 示例目录
if(BUILD_EXAMPLES)
  add_subdirectory(example/go2)
  add_subdirectory(example/go2w)
  add_subdirectory(example/aliengo)
endif()

# 头文件目录
include_directories(include)

# Include standalone CMakeLists
add_subdirectory(src/standalone)
add_subdirectory(tools)

# ROS2依赖 (如果不构建 ROS2 node，注释掉下面所有内容)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 构建 ROS2 node
add_executable(aliengo_node
  src/aliengo_node.cpp
  src/base_robot_control.cpp
  src/sdk1_robot_control.cpp
  src/sdk1_robot_control_him.cpp
  src/sdk1_robot_control_lab.cpp
)
add_executable(go2_node
  src/go2_node.cpp
  src/base_robot_control.cpp
  src/sdk2_robot_control.cpp
  src/sdk2_robot_control_go2.cpp
)
add_executable(go2w_node
  src/go2w_node.cpp
  src/base_robot_control.cpp
  src/sdk2_robot_control.cpp
  src/sdk2_robot_control_go2w.cpp
)

# ament依赖
ament_target_dependencies(aliengo_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
ament_target_dependencies(go2_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
ament_target_dependencies(go2w_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

# 为包含 EP 代码的目标定义编译宏（用于 #ifdef ORT_WITH_*）
if(ENABLE_ORT_CUDA)
  target_compile_definitions(aliengo_node PRIVATE ORT_WITH_CUDA)
  target_compile_definitions(go2_node     PRIVATE ORT_WITH_CUDA)
  target_compile_definitions(go2w_node    PRIVATE ORT_WITH_CUDA)
endif()
if(ENABLE_ORT_TENSORRT)
  target_compile_definitions(aliengo_node PRIVATE ORT_WITH_TENSORRT)
  target_compile_definitions(go2_node     PRIVATE ORT_WITH_TENSORRT)
  target_compile_definitions(go2w_node    PRIVATE ORT_WITH_TENSORRT)
endif()

# 直接从可执行目标进行链接
target_link_libraries(aliengo_node unitree_sdk1)
target_link_libraries(go2_node     unitree_sdk2)
target_link_libraries(go2w_node    unitree_sdk2)

# 运行时库搜索路径
if(UNIX AND NOT APPLE)
  target_link_options(aliengo_node PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
  target_link_options(go2_node     PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
  target_link_options(go2w_node    PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
endif()

# 安装目标
install(TARGETS
  aliengo_node
  go2_node
  go2w_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
