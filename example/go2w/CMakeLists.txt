# Go2W examples using unitree_sdk2 (new SDK)
# These examples demonstrate DDS-based control for Go2W robots

add_executable(go2w_sport_client go2w_sport_client.cpp)
target_link_libraries(go2w_sport_client unitree_sdk2)

add_executable(go2w_stand_example go2w_stand_example.cpp)
target_link_libraries(go2w_stand_example unitree_sdk2)
