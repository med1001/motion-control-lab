cmake_minimum_required(VERSION 3.20)
project(motion_control_lab VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(motion_control
    src/dc_motor.cpp
    src/differential_drive.cpp
    src/pid_controller.cpp
    src/trapezoidal_profile.cpp
    src/waypoint_follower.cpp
)
target_include_directories(motion_control PUBLIC include)

if(MSVC)
    target_compile_options(motion_control PRIVATE /W4 /permissive-)
else()
    target_compile_options(motion_control PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

include(CTest)
