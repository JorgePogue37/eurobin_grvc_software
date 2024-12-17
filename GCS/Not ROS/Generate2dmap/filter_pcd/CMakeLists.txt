cmake_minimum_required(VERSION 3.0.2)
project(filter_pcd)

# find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.8 REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(filter_pcd src/filter_pcd.cpp)
target_link_libraries(filter_pcd ${PCL_LIBRARIES})
target_link_libraries(filter_pcd ${OpenCV_LIBS})
