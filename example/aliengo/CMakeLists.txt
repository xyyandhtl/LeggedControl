# Aliengo examples using unitree_sdk1 (legacy SDK)
# These examples demonstrate UDP-based low-level control for Go1/Aliengo robots

add_executable(aliengo_position example_position_aliengo.cpp)
target_link_libraries(aliengo_position unitree_sdk1)

add_executable(aliengo_velocity example_velocity_aliengo.cpp)
target_link_libraries(aliengo_velocity unitree_sdk1)
