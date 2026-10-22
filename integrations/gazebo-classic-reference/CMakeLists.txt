# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
cmake_minimum_required(VERSION 3.10)
project(mid360_official_reference LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp tf)
add_executable(official-reference reference.cc)
target_include_directories(official-reference PRIVATE /workspace/source/include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
target_link_libraries(official-reference /workspace/out/build/devel/lib/liblivox_laser_simulation.so ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
