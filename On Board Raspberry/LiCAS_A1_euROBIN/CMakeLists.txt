cmake_minimum_required( VERSION 2.8...3.22 )

project( LiCAS_A1 )

add_subdirectory( TaskManager )

add_subdirectory( Kinematics )

add_subdirectory( ServoState )

add_subdirectory( ArmController )

add_subdirectory( ModuleInterface )

add_subdirectory( Main )

