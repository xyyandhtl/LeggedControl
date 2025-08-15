cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(libtorch_infer)

set(CMAKE_PREFIX_PATH "/home/lenovo/Projects/Embodied/src/RobotControl/torchlib")
find_package(Torch REQUIRED)

add_executable(infer main.cpp)
target_link_libraries(infer "${TORCH_LIBRARIES}")
set_property(TARGET infer PROPERTY CXX_STANDARD 14)