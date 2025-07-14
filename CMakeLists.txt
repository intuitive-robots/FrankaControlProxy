project(FrankaControlProxy)
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#include
include_directories(
    include
    include/control_mode
    include/protocol
    include/debugger
)

# dependencies
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(yaml-cpp REQUIRED)


# collect source
file(GLOB_RECURSE SOURCES
    src/*.cpp
)

# Include directories
include_directories(include ${YAML_CPP_INCLUDE_DIR} ${Franka_INCLUDE_DIRS})

# build proxy
add_executable(proxy ${SOURCES})

# connect with dependencies
target_link_libraries(proxy
    ${Franka_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Threads::Threads
)

#include path
target_include_directories(proxy PRIVATE
    ${franka_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
)



