cmake_minimum_required(VERSION 2.8.8)
project(dart_examples)

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

find_package(catkin REQUIRED)
find_package(DART REQUIRED utils-urdf CONFIG)
find_package(aikido REQUIRED COMPONENTS
  constraint
  control_ros
  distance
  planner_ompl
  planner_parabolic
  rviz
  statespace
  trajectory
  util)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

function(add_dart_example target_name)
  add_executable("${target_name}" ${ARGN})
  target_link_libraries("${target_name}"
    "${PROJECT_NAME}_herb"
    ${DART_LIBRARIES}
    ${aikido_LIBRARIES}
  )
endfunction()

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
)

add_library("${PROJECT_NAME}_herb" src/herb.cpp)
target_link_libraries("${PROJECT_NAME}_herb"
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
)

add_dart_example(load_urdf src/load_urdf.cpp)
add_dart_example(ex02_ompl src/ex02_ompl.cpp)
add_dart_example(ex03_ompl src/ex03_ompl.cpp)
add_dart_example(ex04_ompl src/ex04_ompl.cpp)
add_dart_example(ex05_control src/ex05_control.cpp)
