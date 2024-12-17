# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/grvc/euROBIN/LiCAS_A1_euROBIN

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/grvc/euROBIN/LiCAS_A1_euROBIN/build

# Include any dependencies generated for this target.
include ArmController/CMakeFiles/ArmController.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ArmController/CMakeFiles/ArmController.dir/compiler_depend.make

# Include the progress variables for this target.
include ArmController/CMakeFiles/ArmController.dir/progress.make

# Include the compile flags for this target's objects.
include ArmController/CMakeFiles/ArmController.dir/flags.make

ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.o: ArmController/CMakeFiles/ArmController.dir/flags.make
ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.o: /home/grvc/euROBIN/LiCAS_A1_euROBIN/ArmController/ArmController.cpp
ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.o: ArmController/CMakeFiles/ArmController.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/grvc/euROBIN/LiCAS_A1_euROBIN/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.o"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.o -MF CMakeFiles/ArmController.dir/ArmController.cpp.o.d -o CMakeFiles/ArmController.dir/ArmController.cpp.o -c /home/grvc/euROBIN/LiCAS_A1_euROBIN/ArmController/ArmController.cpp

ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ArmController.dir/ArmController.cpp.i"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/grvc/euROBIN/LiCAS_A1_euROBIN/ArmController/ArmController.cpp > CMakeFiles/ArmController.dir/ArmController.cpp.i

ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ArmController.dir/ArmController.cpp.s"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/grvc/euROBIN/LiCAS_A1_euROBIN/ArmController/ArmController.cpp -o CMakeFiles/ArmController.dir/ArmController.cpp.s

# Object files for target ArmController
ArmController_OBJECTS = \
"CMakeFiles/ArmController.dir/ArmController.cpp.o"

# External object files for target ArmController
ArmController_EXTERNAL_OBJECTS =

ArmController/libArmController.a: ArmController/CMakeFiles/ArmController.dir/ArmController.cpp.o
ArmController/libArmController.a: ArmController/CMakeFiles/ArmController.dir/build.make
ArmController/libArmController.a: ArmController/CMakeFiles/ArmController.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/grvc/euROBIN/LiCAS_A1_euROBIN/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libArmController.a"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController && $(CMAKE_COMMAND) -P CMakeFiles/ArmController.dir/cmake_clean_target.cmake
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ArmController.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ArmController/CMakeFiles/ArmController.dir/build: ArmController/libArmController.a
.PHONY : ArmController/CMakeFiles/ArmController.dir/build

ArmController/CMakeFiles/ArmController.dir/clean:
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController && $(CMAKE_COMMAND) -P CMakeFiles/ArmController.dir/cmake_clean.cmake
.PHONY : ArmController/CMakeFiles/ArmController.dir/clean

ArmController/CMakeFiles/ArmController.dir/depend:
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/grvc/euROBIN/LiCAS_A1_euROBIN /home/grvc/euROBIN/LiCAS_A1_euROBIN/ArmController /home/grvc/euROBIN/LiCAS_A1_euROBIN/build /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/ArmController/CMakeFiles/ArmController.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ArmController/CMakeFiles/ArmController.dir/depend
