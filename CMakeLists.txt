cmake_minimum_required(VERSION 3.16)
project(FrankaControlProxy)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------
# C++ settings
# -----------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Build options
# -----------------------------

if(NO_ROBOT_TESTING)
    message(STATUS "NO_ROBOT_TESTING mode is ON")
    add_compile_definitions(NO_ROBOT_TESTING=1)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
else()
    message(STATUS "NO_ROBOT_TESTING mode is OFF")
    add_compile_definitions(NO_ROBOT_TESTING=0)
endif()

# -----------------------------
# Include paths
# -----------------------------
include_directories(
    include
    include/control_mode
    include/protocol
    include/debugger
)

# -----------------------------
# Dependencies
# -----------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Eigen3 3.3.9 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem serialization system)
find_package(yaml-cpp REQUIRED)
find_package(Franka REQUIRED)
# Pub/sub messaging (not on conda-forge); fetched if missing. The pinned commit still includes
# utils/thread_pool.hpp, which upstream deleted in 2a9e4c5 and no longer uses, so the patch drops it.
find_package(zerolancom QUIET)
if(NOT zerolancom_FOUND)
    message(STATUS "zerolancom not found, fetching 68bdabf")
    include(FetchContent)
    set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(zerolancom
        URL https://github.com/xinkai-jiang/ZeroLanCom/archive/68bdabfc7a41e4c71c9ae1b100f892d74e8075c7.tar.gz
        PATCH_COMMAND sed -i /thread_pool.hpp/d include/zerolancom/nodes/zerolancom_node.hpp
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(zerolancom)
    add_library(zerolancom::zerolancom ALIAS zerolancom)
endif()
# Online trajectory generation for the joint reference (conda-forge: ruckig); fetched if missing
find_package(ruckig QUIET)
if(NOT ruckig_FOUND)
    message(STATUS "ruckig not found, fetching v0.14.0")
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(ruckig
        URL https://github.com/pantor/ruckig/archive/refs/tags/v0.14.0.tar.gz)
    FetchContent_MakeAvailable(ruckig)
endif()
if(NO_ROBOT_TESTING)
    find_package(MuJoCo REQUIRED)
    find_package(glfw3 REQUIRED)
endif()
# -----------------------------
# Source collection
# -----------------------------
file(GLOB_RECURSE SOURCES
    src/*.cpp
)

# Exclude mujoco_sim sources when NO_ROBOT_TESTING is OFF
if(NOT NO_ROBOT_TESTING)
    file(GLOB_RECURSE MUJOCO_SIM_SOURCES src/mujoco_sim/*.cpp)
    list(REMOVE_ITEM SOURCES ${MUJOCO_SIM_SOURCES})
endif()

# -----------------------------
# Executable
# -----------------------------
add_executable(proxy src/main.cpp ${SOURCES}
    third_party/robotiq-gripper-interface/src/robotiq_gripper_interface.cc
    third_party/robotiq-gripper-interface/src/helpers.cc
    third_party/robotiq-gripper-interface/src/timeout_reader.cc)

# -----------------------------
# Include dirs & libs
# -----------------------------
target_include_directories(proxy PRIVATE
    include
    third_party/robotiq-gripper-interface/include
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(proxy
    ${YAML_CPP_LIBRARIES}
    zerolancom::zerolancom
    pinocchio::pinocchio
    Eigen3::Eigen
    Franka::Franka
    ruckig::ruckig
)

if(NO_ROBOT_TESTING)
    target_link_libraries(
        proxy     
        MuJoCo::MuJoCo
        glfw
        GL
        X11
    )
endif()
