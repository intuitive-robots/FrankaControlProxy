cmake_minimum_required(VERSION 3.10)
project(FrankaControlProxy)

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
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(yaml-cpp REQUIRED)

# -----------------------------
# Source collection
# -----------------------------
file(GLOB_RECURSE SOURCES
    src/*.cpp
)

# -----------------------------
# Build options
# -----------------------------
option(LOCAL_TESTING "Enable local test mode (skip robot connection etc.)" ON)

if(LOCAL_TESTING)
    message(STATUS "LOCAL_TESTING mode is ON")
    add_compile_definitions(LOCAL_TESTING=1)
else()
    message(STATUS "LOCAL_TESTING mode is OFF")
    add_compile_definitions(LOCAL_TESTING=0)
endif()

# -----------------------------
# Executable
# -----------------------------
add_executable(proxy src/main.cpp ${SOURCES})

# -----------------------------
# Include dirs & libs
# -----------------------------
target_include_directories(proxy PRIVATE
    include
    ${Franka_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(proxy
    ${Franka_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Threads::Threads
)
