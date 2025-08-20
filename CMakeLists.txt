cmake_minimum_required(VERSION 3.10)
project(unitree_control)

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};/home/lenovo/Softwares/libtorch/)

# 选项
option(BUILD_EXAMPLES "Build example programs" ON)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Torch依赖
# find_package(Torch REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
# message("Torch include dirs: ${TORCH_INCLUDE_DIRS}")
# message("Torch libraries: ${TORCH_LIBRARIES}")

# SDK路径
set(UNITREE_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 包含目录
include_directories(
  ${UNITREE_SDK_PATH}/include
  ${UNITREE_SDK_PATH}/include/unitree_sdk2
  ${UNITREE_SDK_PATH}/include/unitree_sdk1
  ${UNITREE_SDK_PATH}/include/ddscxx
  ${UNITREE_SDK_PATH}/include/onnxruntime
  ${UNITREE_SDK_PATH}/src
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
  # ${UNITREE_LIB_PATH}/libonnxruntime_providers_shared.so
  # ${UNITREE_LIB_PATH}/libonnxruntime_providers_cuda.so
  # ${UNITREE_LIB_PATH}/libonnxruntime_providers_tensorrt.so
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

# 示例目录
if(BUILD_EXAMPLES)
  add_subdirectory(example/go2)
  add_subdirectory(example/go2w)
  add_subdirectory(example/aliengo)
endif()

# 直接构建 node，并把实现源码编进可执行文件
add_executable(aliengo_node
  src/aliengo_node.cpp
  src/sdk1/sdk1_robot_control.cpp
)
add_executable(go2_node
  src/go2_node.cpp
  src/sdk2/sdk2_robot_control.cpp
)

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

# 直接从可执行目标进行链接
target_link_libraries(aliengo_node unitree_sdk1)  # 移除 ${TORCH_LIBRARIES} 和重复的 onnxruntime
target_link_libraries(go2_node    unitree_sdk2)

# 运行时库搜索路径
if(UNIX AND NOT APPLE)
  target_link_options(aliengo_node PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
  target_link_options(go2_node PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
endif()

# 安装目标
install(TARGETS
  aliengo_node
  go2_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
