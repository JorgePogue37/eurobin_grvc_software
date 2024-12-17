# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/build

# Include any dependencies generated for this target.
include CMakeFiles/filter_pcd.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/filter_pcd.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/filter_pcd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/filter_pcd.dir/flags.make

CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o: CMakeFiles/filter_pcd.dir/flags.make
CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o: /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/src/filter_pcd.cpp
CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o: CMakeFiles/filter_pcd.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o -MF CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o.d -o CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o -c /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/src/filter_pcd.cpp

CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/src/filter_pcd.cpp > CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.i

CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/src/filter_pcd.cpp -o CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.s

# Object files for target filter_pcd
filter_pcd_OBJECTS = \
"CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o"

# External object files for target filter_pcd
filter_pcd_EXTERNAL_OBJECTS =

filter_pcd: CMakeFiles/filter_pcd.dir/src/filter_pcd.cpp.o
filter_pcd: CMakeFiles/filter_pcd.dir/build.make
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_people.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libboost_system.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libboost_regex.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libqhull.so
filter_pcd: /usr/lib/libOpenNI.so
filter_pcd: /usr/lib/libOpenNI2.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libfreetype.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libz.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libjpeg.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpng.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libtiff.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libexpat.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
filter_pcd: /usr/local/lib/libopencv_gapi.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_stitching.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_aruco.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_bgsegm.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_bioinspired.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_ccalib.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_dnn_objdetect.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_dnn_superres.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_dpm.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_face.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_freetype.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_fuzzy.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_hfs.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_img_hash.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_intensity_transform.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_line_descriptor.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_mcc.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_quality.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_rapid.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_reg.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_rgbd.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_saliency.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_signal.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_stereo.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_structured_light.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_superres.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_surface_matching.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_tracking.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_videostab.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_wechat_qrcode.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_xfeatures2d.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_xobjdetect.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_xphoto.so.4.10.0
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_features.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_search.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_io.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libpcl_common.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libfreetype.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
filter_pcd: /usr/lib/x86_64-linux-gnu/libz.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libGLEW.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libSM.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libICE.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libX11.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libXext.so
filter_pcd: /usr/lib/x86_64-linux-gnu/libXt.so
filter_pcd: /usr/local/lib/libopencv_shape.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_highgui.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_datasets.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_plot.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_text.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_ml.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_phase_unwrapping.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_optflow.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_ximgproc.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_video.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_videoio.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_objdetect.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_calib3d.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_dnn.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_features2d.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_flann.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_photo.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_imgproc.so.4.10.0
filter_pcd: /usr/local/lib/libopencv_core.so.4.10.0
filter_pcd: CMakeFiles/filter_pcd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable filter_pcd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/filter_pcd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/filter_pcd.dir/build: filter_pcd
.PHONY : CMakeFiles/filter_pcd.dir/build

CMakeFiles/filter_pcd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/filter_pcd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/filter_pcd.dir/clean

CMakeFiles/filter_pcd.dir/depend:
	cd /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/build /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/build /home/jorge/Documents/euROBIN/UAVlaunching/filter_pcd/build/CMakeFiles/filter_pcd.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/filter_pcd.dir/depend
