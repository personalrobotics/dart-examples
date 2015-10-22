cmake_minimum_required(VERSION 2.8.8)
project(dart_examples)

find_package(catkin REQUIRED COMPONENTS dart_rviz)
find_package(DART REQUIRED COMPONENTS core)
find_package(r3 REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${r3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(load_urdf src/load_herb.cpp)
target_link_libraries(load_urdf
  ${DART_LIBRARIES}
  ${r3_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS load_herb
  EXPORT "${PROJECT_NAME}Targets"
  RUNTIME DESTINATION "bin"
)
