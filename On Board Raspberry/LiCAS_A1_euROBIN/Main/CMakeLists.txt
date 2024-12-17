cmake_minimum_required( VERSION 2.8...3.22 )

include_directories( ${LICAS_A1_SOURCE_DIR}/Threads )
include_directories( ${LICAS_A1_SOURCE_DIR}/Structures )
include_directories( ${LICAS_A1_SOURCE_DIR}/TaskManager )
include_directories( ${LICAS_A1_SOURCE_DIR}/Kinematics )
include_directories( ${LICAS_A1_SOURCE_DIR}/ArmController )
include_directories( ${LICAS_A1_SOURCE_DIR}/ServoState )
include_directories( ${LICAS_A1_SOURCE_DIR}/ModuleInterface )

add_executable( LiCAS_A1 Main.cpp )

# Link the executable to the libraries. The order of the libraries is important.
target_link_libraries( LiCAS_A1 Kinematics ServoState ArmController TaskManager -pthread )

