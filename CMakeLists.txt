cmake_minimum_required(VERSION 2.8.8)
project(dart_examples)

find_package(catkin REQUIRED)
find_package(DART REQUIRED utils utils-urdf CONFIG)
find_package(aikido REQUIRED COMPONENTS
  constraint
  distance
  planner_mintos
  planner_ompl
  planner_parabolic
  rviz
  statespace
  util
)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

function(add_dart_example target_name)
  add_executable("${target_name}" ${ARGN})
  target_link_libraries("${target_name}"
    "${PROJECT_NAME}_herb"
    "${PROJECT_NAME}_distanceconstraint"
    "${PROJECT_NAME}_configurationtestable"
    ${DART_LIBRARIES}
    ${aikido_LIBRARIES}
  )
endfunction()

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
)

add_library("${PROJECT_NAME}_herb" SHARED src/herb.cpp)
target_link_libraries("${PROJECT_NAME}_herb"
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  "${PROJECT_NAME}_distanceconstraint"
  "${PROJECT_NAME}_configurationtestable"
)

add_library("${PROJECT_NAME}_distanceconstraint" SHARED
  src/DistanceConstraint.cpp)
target_link_libraries("${PROJECT_NAME}_distanceconstraint"
  ${aikido_LIBRARIES})

add_library("${PROJECT_NAME}_configurationtestable" SHARED
  src/ConfigurationTestable.cpp)
target_link_libraries("${PROJECT_NAME}_configurationtestable"
  ${aikido_LIBRARIES})

add_dart_example(load_urdf src/load_urdf.cpp)
add_dart_example(ex02_ompl src/ex02_ompl.cpp)
add_dart_example(ex03_ompl src/ex03_ompl.cpp)
add_dart_example(ex04_ompl src/ex04_ompl.cpp)
add_dart_example(ex05_mintos src/ex05_mintos.cpp)
