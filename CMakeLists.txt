cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tst)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (tst floorplan.cpp)
target_link_libraries (tst ${PCL_LIBRARIES})
target_link_libraries (tst ${OpenCV_LIBS})