cmake_minimum_required( VERSION 2.8...3.22 )

add_library( ArmController ArmController.h ArmController.cpp )

target_link_libraries( ArmController ServoState Kinematics -pthread )

