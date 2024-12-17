cmake_minimum_required(VERSION 3.0.2)
project(target_point_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# find_package(system_lib REQUIRED)
# include_directories(${system_lib_INCLUDE_DIRS})

add_library(ModuleInterface /home/jorge/catkin_ws/src/target_point_publisher/include/ModuleInterface/ModuleInterface.cpp)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_generator
  CATKIN_DEPENDS nav_msgs roscpp std_msgs tf2_geometry_msgs geometry_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${MODULETEMPLATE_SOURCE_DIR}/ModuleInterface
)

add_executable(${PROJECT_NAME}_node src/targetPoint_publisher.cpp)
add_executable(path_generator_node src/path_generator.cpp src/a_star.cpp)
add_executable(aruco_guidance_node src/aruco_guidance.cpp)
add_executable(ADROM2ROScommunication_node src/ADROM2ROScommunication.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aruco_guidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ADROM2ROScommunication_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

target_link_libraries(path_generator_node
  ${catkin_LIBRARIES}
)

target_link_libraries(aruco_guidance_node
  ${catkin_LIBRARIES}
  -pthread
)

 target_link_libraries(ADROM2ROScommunication_node
  ${catkin_LIBRARIES}
  ModuleInterface
  -pthread
)
