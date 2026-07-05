project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

# 输出二进制到 arm_task/bin/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../bin)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

add_executable(d1_disable src/d1_disable.cpp src/msg/ArmString_.cpp)
add_executable(d1_enable src/d1_enable.cpp src/msg/ArmString_.cpp)
add_executable(d1_home src/d1_home.cpp src/msg/ArmString_.cpp)
add_executable(d1_move_single src/d1_move_single.cpp src/msg/ArmString_.cpp)
add_executable(d1_move_multiple src/d1_move_multiple.cpp src/msg/ArmString_.cpp)
add_executable(d1_get_arm_joint_angle src/d1_get_arm_joint_angle.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
add_executable(d1_safe_fold src/d1_safe_fold.cpp src/msg/ArmString_.cpp)
