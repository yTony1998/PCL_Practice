# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/build

# Include any dependencies generated for this target.
include CMakeFiles/kd_tree.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kd_tree.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kd_tree.dir/flags.make

CMakeFiles/kd_tree.dir/kd_tree.cpp.o: CMakeFiles/kd_tree.dir/flags.make
CMakeFiles/kd_tree.dir/kd_tree.cpp.o: ../kd_tree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/kd_tree.dir/kd_tree.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kd_tree.dir/kd_tree.cpp.o -c /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/kd_tree.cpp

CMakeFiles/kd_tree.dir/kd_tree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kd_tree.dir/kd_tree.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/kd_tree.cpp > CMakeFiles/kd_tree.dir/kd_tree.cpp.i

CMakeFiles/kd_tree.dir/kd_tree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kd_tree.dir/kd_tree.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/kd_tree.cpp -o CMakeFiles/kd_tree.dir/kd_tree.cpp.s

CMakeFiles/kd_tree.dir/kd_tree.cpp.o.requires:

.PHONY : CMakeFiles/kd_tree.dir/kd_tree.cpp.o.requires

CMakeFiles/kd_tree.dir/kd_tree.cpp.o.provides: CMakeFiles/kd_tree.dir/kd_tree.cpp.o.requires
	$(MAKE) -f CMakeFiles/kd_tree.dir/build.make CMakeFiles/kd_tree.dir/kd_tree.cpp.o.provides.build
.PHONY : CMakeFiles/kd_tree.dir/kd_tree.cpp.o.provides

CMakeFiles/kd_tree.dir/kd_tree.cpp.o.provides.build: CMakeFiles/kd_tree.dir/kd_tree.cpp.o


# Object files for target kd_tree
kd_tree_OBJECTS = \
"CMakeFiles/kd_tree.dir/kd_tree.cpp.o"

# External object files for target kd_tree
kd_tree_EXTERNAL_OBJECTS =

kd_tree: CMakeFiles/kd_tree.dir/kd_tree.cpp.o
kd_tree: CMakeFiles/kd_tree.dir/build.make
kd_tree: /usr/lib/libpcl_visualization.so
kd_tree: /usr/lib/x86_64-linux-gnu/libboost_system.so
kd_tree: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
kd_tree: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
kd_tree: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
kd_tree: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
kd_tree: /usr/lib/x86_64-linux-gnu/libboost_regex.so
kd_tree: /usr/lib/libOpenNI.so
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libfreetype.so
kd_tree: /usr/lib/x86_64-linux-gnu/libz.so
kd_tree: /usr/lib/x86_64-linux-gnu/libjpeg.so
kd_tree: /usr/lib/x86_64-linux-gnu/libpng.so
kd_tree: /usr/lib/x86_64-linux-gnu/libtiff.so
kd_tree: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
kd_tree: /usr/lib/libpcl_io.so
kd_tree: /usr/lib/libpcl_search.so
kd_tree: /usr/lib/libpcl_octree.so
kd_tree: /usr/lib/libpcl_kdtree.so
kd_tree: /usr/lib/libpcl_common.so
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libfreetype.so
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0
kd_tree: /usr/lib/x86_64-linux-gnu/libz.so
kd_tree: /usr/lib/x86_64-linux-gnu/libGLU.so
kd_tree: /usr/lib/x86_64-linux-gnu/libGL.so
kd_tree: /usr/lib/x86_64-linux-gnu/libSM.so
kd_tree: /usr/lib/x86_64-linux-gnu/libICE.so
kd_tree: /usr/lib/x86_64-linux-gnu/libX11.so
kd_tree: /usr/lib/x86_64-linux-gnu/libXext.so
kd_tree: /usr/lib/x86_64-linux-gnu/libXt.so
kd_tree: CMakeFiles/kd_tree.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable kd_tree"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kd_tree.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kd_tree.dir/build: kd_tree

.PHONY : CMakeFiles/kd_tree.dir/build

CMakeFiles/kd_tree.dir/requires: CMakeFiles/kd_tree.dir/kd_tree.cpp.o.requires

.PHONY : CMakeFiles/kd_tree.dir/requires

CMakeFiles/kd_tree.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kd_tree.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kd_tree.dir/clean

CMakeFiles/kd_tree.dir/depend:
	cd /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/build /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/build /home/yingyue/Desktop/PCL_myPractice/PCL_Practice/kd_tree/build/CMakeFiles/kd_tree.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kd_tree.dir/depend

