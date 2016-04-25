cmake_minimum_required(VERSION 2.8.8)
project(dart_examples)

find_package(catkin REQUIRED)
find_package(DART REQUIRED COMPONENTS core)
find_package(aikido REQUIRED COMPONENTS
  constraint
  distance
  planner_ompl
  planner_parabolic
  statespace
  util)
find_package(aikido_rviz REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

function(add_dart_example target_name)
  add_executable("${target_name}" ${ARGN})
  target_link_libraries("${target_name}"
    ${DART_LIBRARIES}
    ${aikido_LIBRARIES}
    ${aikido_rviz_LIBRARIES}
  )
endfunction()

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
  ${aikido_rviz_INCLUDE_DIRS}
)

add_dart_example(load_urdf src/load_urdf.cpp)
#add_dart_example(ex02_ompl src/ex02_ompl.cpp)
add_dart_example(ex03_ompl src/ex03_ompl.cpp)
