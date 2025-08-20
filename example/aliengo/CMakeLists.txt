# Aliengo examples using unitree_sdk1 (legacy SDK)
# These examples demonstrate UDP-based low-level control for Go1/Aliengo robots

file(GLOB EXAMPLES_SRC "*.cpp")

foreach(SRC_FILE ${EXAMPLES_SRC})
    get_filename_component(EXEC_NAME ${SRC_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SRC_FILE})
    target_link_libraries(${EXEC_NAME} unitree_sdk1)
endforeach()
