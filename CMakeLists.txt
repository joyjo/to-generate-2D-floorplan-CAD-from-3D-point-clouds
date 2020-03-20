cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(floorplan)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (floorplan floorplan.cpp)
target_link_libraries (floorplan ${PCL_LIBRARIES})
target_link_libraries (floorplan ${OpenCV_LIBS})