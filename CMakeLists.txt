cmake_minimum_required(VERSION 3.8)
project(unitree_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 设置Unitree SDK路径
set(UNITREE_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 包含目录
include_directories(
  ${UNITREE_SDK_PATH}/include
  ${UNITREE_SDK_PATH}/include/unitree_sdk2
  ${UNITREE_SDK_PATH}/include/unitree_sdk1
  ${UNITREE_SDK_PATH}/include/ddscxx
#   ${UNITREE_SDK_PATH}/include/dds
)

# 根据架构选择库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(UNITREE_LIB_PATH ${UNITREE_SDK_PATH}/lib/aarch64)
else()
  set(UNITREE_LIB_PATH ${UNITREE_SDK_PATH}/lib/x86_64)
endif()

# 链接目录
link_directories(${UNITREE_LIB_PATH})

# 定义库的别名，供子目录使用
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
  pthread
  dl
)

add_library(unitree_sdk1 INTERFACE)
target_include_directories(unitree_sdk1 INTERFACE
  ${UNITREE_SDK_PATH}/include
  ${UNITREE_SDK_PATH}/include/unitree_sdk1
)
target_link_libraries(unitree_sdk1 INTERFACE
  ${UNITREE_LIB_PATH}/libunitree_legged_sdk.so
  # pthread
  # dl
)

# 创建可执行文件
add_executable(sdk2_velocity_node src/sdk2_velocity_node.cpp)
add_executable(sdk1_velocity_node src/sdk1_velocity_node.cpp)

# 添加示例程序支持
# 注意：示例程序需要 unitree_sdk1 和 unitree_sdk2 库
# 如果只需要 ROS 2 功能，可以注释掉以下部分

# 包含示例目录
add_subdirectory(example/go2)
add_subdirectory(example/go2w)
add_subdirectory(example/aliengo)

# ament依赖（自动注入包含目录与链接库）
ament_target_dependencies(sdk2_velocity_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

ament_target_dependencies(sdk1_velocity_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

# 链接库（使用已定义的库别名）
target_link_libraries(sdk2_velocity_node unitree_sdk2)
target_link_libraries(sdk1_velocity_node unitree_sdk1)

# 运行时库搜索路径（避免运行时找不到Unitree .so）
if(UNIX AND NOT APPLE)
  target_link_options(sdk2_velocity_node PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
  target_link_options(sdk1_velocity_node PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
endif()

# 安装目标
install(TARGETS
  sdk2_velocity_node
  sdk1_velocity_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装示例程序
install(TARGETS  
  # Go2W examples
  go2w_sport_client
  go2w_stand_example
  # Aliengo examples
  aliengo_position
  aliengo_velocity
  DESTINATION lib/${PROJECT_NAME}/examples
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_package()
