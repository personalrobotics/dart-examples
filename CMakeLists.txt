cmake_minimum_required(VERSION 2.8.8)
project(dart_examples)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake") 

find_package(catkin REQUIRED COMPONENTS aikido_rviz aikido_perception )
find_package(DART REQUIRED COMPONENTS core)
find_package(aikido REQUIRED)
find_package(ASSIMP REQUIRED)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

add_executable(load_urdf
  src/load_urdf.cpp
)
target_link_libraries(load_urdf
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

install(TARGETS load_urdf
  EXPORT "${PROJECT_NAME}Targets"
  RUNTIME DESTINATION "bin"
)

add_executable(test_perception
  src/test_perception.cpp
)
target_link_libraries(test_perception
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

install(TARGETS test_perception
  EXPORT "${PROJECT_NAME}Targets"
  RUNTIME DESTINATION "bin"
)