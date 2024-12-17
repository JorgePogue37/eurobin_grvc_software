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
include Kinematics/CMakeFiles/Kinematics.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include Kinematics/CMakeFiles/Kinematics.dir/compiler_depend.make

# Include the progress variables for this target.
include Kinematics/CMakeFiles/Kinematics.dir/progress.make

# Include the compile flags for this target's objects.
include Kinematics/CMakeFiles/Kinematics.dir/flags.make

Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.o: Kinematics/CMakeFiles/Kinematics.dir/flags.make
Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.o: /home/grvc/euROBIN/LiCAS_A1_euROBIN/Kinematics/Kinematics.cpp
Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.o: Kinematics/CMakeFiles/Kinematics.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/grvc/euROBIN/LiCAS_A1_euROBIN/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.o"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.o -MF CMakeFiles/Kinematics.dir/Kinematics.cpp.o.d -o CMakeFiles/Kinematics.dir/Kinematics.cpp.o -c /home/grvc/euROBIN/LiCAS_A1_euROBIN/Kinematics/Kinematics.cpp

Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kinematics.dir/Kinematics.cpp.i"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/grvc/euROBIN/LiCAS_A1_euROBIN/Kinematics/Kinematics.cpp > CMakeFiles/Kinematics.dir/Kinematics.cpp.i

Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kinematics.dir/Kinematics.cpp.s"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/grvc/euROBIN/LiCAS_A1_euROBIN/Kinematics/Kinematics.cpp -o CMakeFiles/Kinematics.dir/Kinematics.cpp.s

# Object files for target Kinematics
Kinematics_OBJECTS = \
"CMakeFiles/Kinematics.dir/Kinematics.cpp.o"

# External object files for target Kinematics
Kinematics_EXTERNAL_OBJECTS =

Kinematics/libKinematics.a: Kinematics/CMakeFiles/Kinematics.dir/Kinematics.cpp.o
Kinematics/libKinematics.a: Kinematics/CMakeFiles/Kinematics.dir/build.make
Kinematics/libKinematics.a: Kinematics/CMakeFiles/Kinematics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/grvc/euROBIN/LiCAS_A1_euROBIN/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libKinematics.a"
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics && $(CMAKE_COMMAND) -P CMakeFiles/Kinematics.dir/cmake_clean_target.cmake
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Kinematics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Kinematics/CMakeFiles/Kinematics.dir/build: Kinematics/libKinematics.a
.PHONY : Kinematics/CMakeFiles/Kinematics.dir/build

Kinematics/CMakeFiles/Kinematics.dir/clean:
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics && $(CMAKE_COMMAND) -P CMakeFiles/Kinematics.dir/cmake_clean.cmake
.PHONY : Kinematics/CMakeFiles/Kinematics.dir/clean

Kinematics/CMakeFiles/Kinematics.dir/depend:
	cd /home/grvc/euROBIN/LiCAS_A1_euROBIN/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/grvc/euROBIN/LiCAS_A1_euROBIN /home/grvc/euROBIN/LiCAS_A1_euROBIN/Kinematics /home/grvc/euROBIN/LiCAS_A1_euROBIN/build /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics /home/grvc/euROBIN/LiCAS_A1_euROBIN/build/Kinematics/CMakeFiles/Kinematics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Kinematics/CMakeFiles/Kinematics.dir/depend

