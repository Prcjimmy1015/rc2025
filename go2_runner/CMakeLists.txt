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

# ─────────── 项目根目录 (rc2025/) 绝对路径，供 ONNX 模型加载 ──────────
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)
message(STATUS "Project root: ${PROJECT_ROOT}")

# ─────────── test_task (调试入口，无摄像头依赖) ──────────
add_executable(test_task
    test_task.cpp
)
set_target_properties(test_task PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH};${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}"
    LINK_FLAGS "-Wl,--disable-new-dtags"
)
target_include_directories(test_task PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(test_task PRIVATE
    PROJECT_ROOT="${PROJECT_ROOT}"
)
target_link_libraries(test_task
    PRIVATE
        unitree_sdk2
        ddscxx
        ddsc
        pthread
)

# ─────────── action_test (单独动作测试) ──────────
add_executable(action_test
    action_test.cpp
    globals.cpp
    callbacks.cpp
    utils.cpp
    app_runtime.cpp
)
target_compile_definitions(action_test PRIVATE
    PROJECT_ROOT="${PROJECT_ROOT}"
)
set_target_properties(action_test PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH};${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}"
    LINK_FLAGS "-Wl,--disable-new-dtags"
)
target_link_libraries(action_test
    PRIVATE
        ${OpenCV_LIBS}
        unitree_sdk2
        ddscxx
        ddsc
        pthread
)
