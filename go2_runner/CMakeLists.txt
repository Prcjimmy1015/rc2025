cmake_minimum_required(VERSION 3.16)
project(rc2025 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# ─────────── OpenCV (required) ───────────
find_package(OpenCV REQUIRED)

# ─────────── ROS2 dependencies ───────────
find_package(std_msgs REQUIRED)

# ─────────── Locate Unitree SDK2 ─────────
# Priority:  (1) -DUNITREE_SDK2_DIR=…  (2) $ENV{UNITREE_SDK2_DIR}  (3) $HOME/unitree_sdk2
if(NOT UNITREE_SDK2_DIR)
    if(DEFINED ENV{UNITREE_SDK2_DIR})
        set(UNITREE_SDK2_DIR $ENV{UNITREE_SDK2_DIR})
    else()
        set(UNITREE_SDK2_DIR "$ENV{HOME}/unitree_sdk2")
    endif()
endif()
message(STATUS "Using UNITREE_SDK2_DIR = ${UNITREE_SDK2_DIR}")

# Headers
include_directories(
    ${UNITREE_SDK2_DIR}/include
    ${UNITREE_SDK2_DIR}/thirdparty/include       # DDS core
    ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
)

# ── Auto-detect CPU architecture ──────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(UNITREE_ARCH "aarch64")
else()
    set(UNITREE_ARCH "x86_64")
endif()
message(STATUS "Unitree SDK architecture = ${UNITREE_ARCH}")

# Libraries
link_directories(
    ${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}
    ${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH}
)

# ─────────── rc2025_run (主入口) ──────────────
add_executable(rc2025_run
    main.cpp
    globals.cpp
    callbacks.cpp
    utils.cpp
    visualizer.cpp
    aruco_server.cpp
    app_runtime.cpp
    line_follow.cpp
    cases/case0.cpp
    cases/case1.cpp
    cases/case2.cpp
    cases/case3.cpp
)
set_target_properties(rc2025_run PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH};${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}"
    LINK_FLAGS "-Wl,--disable-new-dtags"
)
target_link_libraries(rc2025_run
    PRIVATE
        ${OpenCV_LIBS}
        unitree_sdk2
        ddscxx
        ddsc
        pthread
)

# ─────────── test_task (调试入口，比赛前移除) ──────────
add_executable(test_task
    test_task.cpp
    globals.cpp
    callbacks.cpp
    utils.cpp
    aruco_server.cpp
    app_runtime.cpp
)
set_target_properties(test_task PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH};${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}"
    LINK_FLAGS "-Wl,--disable-new-dtags"
)
target_link_libraries(test_task
    PRIVATE
        ${OpenCV_LIBS}
        unitree_sdk2
        ddscxx
        ddsc
        pthread
)