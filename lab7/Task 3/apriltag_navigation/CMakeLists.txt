cmake_minimum_required(VERSION 3.0.2)
project(apriltag_navigation)
set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  apriltag_ros
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf apriltag_ros
)



include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB src_files
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME}_node src/apriltag_nav.cpp ${src_files})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )