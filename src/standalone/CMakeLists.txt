# Standalone executables
add_executable(aliengo_him_standalone
  aliengo_him.cpp
  ../base_robot_control.cpp
  ../sdk1_robot_control.cpp
  ../sdk1_robot_control_him.cpp
)
add_executable(aliengo_lab_standalone
  aliengo_lab.cpp
  ../base_robot_control.cpp
  ../sdk1_robot_control.cpp
  ../sdk1_robot_control_lab.cpp
)
add_executable(go2_standalone
  go2.cpp
  ../base_robot_control.cpp
  ../sdk2_robot_control.cpp
  ../sdk2_robot_control_go2.cpp
)
add_executable(go2w_standalone
  go2w.cpp
  ../base_robot_control.cpp
  ../sdk2_robot_control.cpp
  ../sdk2_robot_control_go2w.cpp
)

# Link libraries
target_link_libraries(aliengo_him_standalone unitree_sdk1)
target_link_libraries(aliengo_lab_standalone unitree_sdk1)
target_link_libraries(go2_standalone unitree_sdk2)
target_link_libraries(go2w_standalone unitree_sdk2)

# Runtime library search path
if(UNIX AND NOT APPLE)
  target_link_options(aliengo_him_standalone PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
  target_link_options(aliengo_lab_standalone PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
  target_link_options(go2_standalone PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
  target_link_options(go2w_standalone PRIVATE -Wl,-rpath,${UNITREE_LIB_PATH})
endif()

# Compile definitions for ORT providers
if(ENABLE_ORT_CUDA)
  target_compile_definitions(aliengo_him_standalone PRIVATE ORT_WITH_CUDA)
  target_compile_definitions(aliengo_lab_standalone PRIVATE ORT_WITH_CUDA)
  target_compile_definitions(go2_standalone PRIVATE ORT_WITH_CUDA)
  target_compile_definitions(go2w_standalone PRIVATE ORT_WITH_CUDA)
endif()
if(ENABLE_ORT_TENSORRT)
  target_compile_definitions(aliengo_him_standalone PRIVATE ORT_WITH_TENSORRT)
  target_compile_definitions(aliengo_lab_standalone PRIVATE ORT_WITH_TENSORRT)
  target_compile_definitions(go2_standalone PRIVATE ORT_WITH_TENSORRT)
  target_compile_definitions(go2w_standalone PRIVATE ORT_WITH_TENSORRT)
endif()

# Set output directory for standalone executables
set(STANDALONE_OUTPUT_DIR ${CMAKE_BINARY_DIR}/standalone)
set_target_properties(aliengo_him_standalone PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${STANDALONE_OUTPUT_DIR})
set_target_properties(aliengo_lab_standalone PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${STANDALONE_OUTPUT_DIR})
set_target_properties(go2_standalone PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${STANDALONE_OUTPUT_DIR})
set_target_properties(go2w_standalone PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${STANDALONE_OUTPUT_DIR})