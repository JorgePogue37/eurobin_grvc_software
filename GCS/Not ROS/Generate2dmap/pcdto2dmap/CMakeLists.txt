cmake_minimum_required(VERSION 3.0.2)
project(pcd_to_2dmap)

# find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.8 REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_to_2dmap src/pcd_to_2dmap.cpp)
target_link_libraries(pcd_to_2dmap ${PCL_LIBRARIES})
target_link_libraries(pcd_to_2dmap ${OpenCV_LIBS})
