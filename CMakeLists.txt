cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pose_estimation)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")

find_package(PCL 1.7 REQUIRED)
find_package(yaml-cpp REQUIRED)
# find glog
find_package(Glog REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})

add_executable (track src/test_mutiple.cpp src/model_match.cpp src/model_track.cpp)
target_link_libraries (track ${PCL_LIBRARIES} glog yaml-cpp)

add_executable (match src/test_single.cpp src/model_match.cpp)
target_link_libraries (match ${PCL_LIBRARIES} glog yaml-cpp)
