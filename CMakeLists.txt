cmake_minimum_required(VERSION 3.5)
project(LOAM_Feature_Vis)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(.)

AUX_SOURCE_DIRECTORY(. DIR_SRCS)

add_executable(${PROJECT_NAME}
        ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})