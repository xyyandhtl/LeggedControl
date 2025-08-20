# Go2W examples using unitree_sdk2 (new SDK)
# These examples demonstrate DDS-based control for Go2W robots

file(GLOB EXAMPLES_SRC "*.cpp")

foreach(SRC_FILE ${EXAMPLES_SRC})
    get_filename_component(EXEC_NAME ${SRC_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SRC_FILE})
    target_link_libraries(${EXEC_NAME} unitree_sdk2)
endforeach()
