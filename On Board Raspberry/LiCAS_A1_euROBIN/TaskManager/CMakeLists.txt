cmake_minimum_required( VERSION 2.8...3.22 )

add_library( TaskManager TaskManager.h TaskManager.cpp TaskManager2.cpp )

target_link_libraries( TaskManager ArmController ModuleInterface -pthread )

