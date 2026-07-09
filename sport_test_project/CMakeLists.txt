cmake_minimum_required(VERSION 3.16)
project(sport_test LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# ─────────── Locate Unitree SDK2 ─────────
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
    ${UNITREE_SDK2_DIR}/thirdparty/include
    ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
)

# Libraries (auto-detect architecture)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(UNITREE_ARCH "aarch64")
else()
    set(UNITREE_ARCH "x86_64")
endif()
message(STATUS "Using Unitree SDK arch = ${UNITREE_ARCH}")

link_directories(
    ${UNITREE_SDK2_DIR}/lib/${UNITREE_ARCH}
    ${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_ARCH}
)

# ─────────── Executable ──────────────
add_executable(sport_test sport_test.cpp)

target_link_libraries(sport_test
    PRIVATE
        unitree_sdk2
        ddscxx
        ddsc
        pthread
)