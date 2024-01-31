cmake_minimum_required(VERSION 3.12)

# -- TASK1
#project(SergioEstevaoProgram)

#find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)

#set(SOURCES sergio_estevao.cpp)
#include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
#add_executable(SergioEstevaoProgram ${SOURCES})
#target_link_libraries(SergioEstevaoProgram Eigen3::Eigen ${OpenCV_LIBS})


# -- TASK2
project(ObjectDetection)

find_package(OpenCV REQUIRED)

set(SOURCES objectDetection.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(ObjectDetection ${SOURCES})
target_link_libraries(ObjectDetection ${OpenCV_LIBS})