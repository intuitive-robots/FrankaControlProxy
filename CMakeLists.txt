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

if(DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
    set(Boost_NO_SYSTEM_PATHS ON) 
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
find_package(zerolancom REQUIRED)
find_package(Franka REQUIRED)
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
