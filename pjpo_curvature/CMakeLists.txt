cmake_minimum_required(VERSION 3.0.2)
project(pjpo_curvature)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-std=c++11 -O3)

set(OsqpEigen_DIR "/usr/local/lib/cmake/OsqpEigen/")
set(Eigen3_DIR "/usr/lib/cmake/eigen3/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  Eigen3 REQUIRED
  OsqpEigen REQUIRED
)

add_message_files(
  FILES
  CenterLinePoint.msg
  CenterLine.msg
  DynamicObstacle.msg
  DynamicObstacles.msg
  Obstacles.msg
  DynamicTrajectoryPoint.msg
  FrenetPoint.msg
  FrenetPath.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pjpo_curvature
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

option(WITH_HSL "Compile solver with coinhsl" ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(HSL)

if(NOT ${HSL_FOUND})
  set(WITH_HSL OFF)
endif()

if(WITH_HSL)
  add_definitions(-DWITH_HSL)
endif()

set(SRC
  src/pjpo_curvature/math/aabox2d.cpp
  src/pjpo_curvature/math/box2d.cpp
  src/pjpo_curvature/math/line_segment2d.cpp
  src/pjpo_curvature/math/math_utils.cpp
  src/pjpo_curvature/math/polygon2d.cpp
  src/pjpo_curvature/math/vec2d.cpp
  src/pjpo_curvature/pjpo_curvature.cpp
  src/pjpo_curvature/discretized_trajectory.cpp
  src/pjpo_curvature/environment.cpp
  src/pjpo_curvature/dp_planner.cpp
  src/pjpo_curvature/visualization/plot.cpp
  src/pjpo_curvature/visualization/color.cpp)

add_library(${PROJECT_NAME} ${SRC})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} OsqpEigen::OsqpEigen)

add_executable(${PROJECT_NAME}_node src/pjpo_curvature_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  OsqpEigen::OsqpEigen
)

