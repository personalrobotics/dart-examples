cmake_minimum_required(VERSION 2.8.8)
project(dart_examples)

find_package(catkin REQUIRED)
find_package(DART 6.3 REQUIRED utils)
find_package(aikido 0.2.0 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    io
    perception
    planner_ompl
    planner_parabolic
    planner_vectorfield
    rviz
    statespace
)

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
