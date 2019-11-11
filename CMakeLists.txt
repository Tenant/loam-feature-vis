cmake_minimum_required(VERSION 3.5)
project(LOAM_Feature_Vis)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Debug)

find_package(Boost 1.6 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories( "/usr/include/eigen3" )

include_directories(.
        ./loam_velodyne)

AUX_SOURCE_DIRECTORY(. DIR_ROOT)
AUX_SOURCE_DIRECTORY(./loam_velodyne DIR_LOAM)



add_executable(${PROJECT_NAME}
        ${DIR_ROOT}
        ${DIR_LOAM})
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES})