# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zhu/irdv_sim20160811/irdv_sim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cpp.o: ../src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/main.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/main.cpp.o -c /home/zhu/irdv_sim20160811/irdv_sim/src/main.cpp

CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/zhu/irdv_sim20160811/irdv_sim/src/main.cpp > CMakeFiles/main.dir/src/main.cpp.i

CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/zhu/irdv_sim20160811/irdv_sim/src/main.cpp -o CMakeFiles/main.dir/src/main.cpp.s

CMakeFiles/main.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/main.dir/src/main.cpp.o.requires

CMakeFiles/main.dir/src/main.cpp.o.provides: CMakeFiles/main.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/src/main.cpp.o.provides

CMakeFiles/main.dir/src/main.cpp.o.provides.build: CMakeFiles/main.dir/src/main.cpp.o

CMakeFiles/main.dir/src/basefunc.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/basefunc.cpp.o: ../src/basefunc.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/main.dir/src/basefunc.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/basefunc.cpp.o -c /home/zhu/irdv_sim20160811/irdv_sim/src/basefunc.cpp

CMakeFiles/main.dir/src/basefunc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/basefunc.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/zhu/irdv_sim20160811/irdv_sim/src/basefunc.cpp > CMakeFiles/main.dir/src/basefunc.cpp.i

CMakeFiles/main.dir/src/basefunc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/basefunc.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/zhu/irdv_sim20160811/irdv_sim/src/basefunc.cpp -o CMakeFiles/main.dir/src/basefunc.cpp.s

CMakeFiles/main.dir/src/basefunc.cpp.o.requires:
.PHONY : CMakeFiles/main.dir/src/basefunc.cpp.o.requires

CMakeFiles/main.dir/src/basefunc.cpp.o.provides: CMakeFiles/main.dir/src/basefunc.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/basefunc.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/src/basefunc.cpp.o.provides

CMakeFiles/main.dir/src/basefunc.cpp.o.provides.build: CMakeFiles/main.dir/src/basefunc.cpp.o

CMakeFiles/main.dir/src/calib/stereocalib.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/calib/stereocalib.cpp.o: ../src/calib/stereocalib.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/main.dir/src/calib/stereocalib.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/calib/stereocalib.cpp.o -c /home/zhu/irdv_sim20160811/irdv_sim/src/calib/stereocalib.cpp

CMakeFiles/main.dir/src/calib/stereocalib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/calib/stereocalib.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/zhu/irdv_sim20160811/irdv_sim/src/calib/stereocalib.cpp > CMakeFiles/main.dir/src/calib/stereocalib.cpp.i

CMakeFiles/main.dir/src/calib/stereocalib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/calib/stereocalib.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/zhu/irdv_sim20160811/irdv_sim/src/calib/stereocalib.cpp -o CMakeFiles/main.dir/src/calib/stereocalib.cpp.s

CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.requires:
.PHONY : CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.requires

CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.provides: CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.provides

CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.provides.build: CMakeFiles/main.dir/src/calib/stereocalib.cpp.o

CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o: ../src/rectify/stereorectify.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o -c /home/zhu/irdv_sim20160811/irdv_sim/src/rectify/stereorectify.cpp

CMakeFiles/main.dir/src/rectify/stereorectify.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/rectify/stereorectify.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/zhu/irdv_sim20160811/irdv_sim/src/rectify/stereorectify.cpp > CMakeFiles/main.dir/src/rectify/stereorectify.cpp.i

CMakeFiles/main.dir/src/rectify/stereorectify.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/rectify/stereorectify.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/zhu/irdv_sim20160811/irdv_sim/src/rectify/stereorectify.cpp -o CMakeFiles/main.dir/src/rectify/stereorectify.cpp.s

CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.requires:
.PHONY : CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.requires

CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.provides: CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.provides

CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.provides.build: CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o

CMakeFiles/main.dir/src/stereoreconstruction.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/stereoreconstruction.cpp.o: ../src/stereoreconstruction.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/main.dir/src/stereoreconstruction.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/stereoreconstruction.cpp.o -c /home/zhu/irdv_sim20160811/irdv_sim/src/stereoreconstruction.cpp

CMakeFiles/main.dir/src/stereoreconstruction.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/stereoreconstruction.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/zhu/irdv_sim20160811/irdv_sim/src/stereoreconstruction.cpp > CMakeFiles/main.dir/src/stereoreconstruction.cpp.i

CMakeFiles/main.dir/src/stereoreconstruction.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/stereoreconstruction.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/zhu/irdv_sim20160811/irdv_sim/src/stereoreconstruction.cpp -o CMakeFiles/main.dir/src/stereoreconstruction.cpp.s

CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.requires:
.PHONY : CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.requires

CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.provides: CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.provides

CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.provides.build: CMakeFiles/main.dir/src/stereoreconstruction.cpp.o

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/main.cpp.o" \
"CMakeFiles/main.dir/src/basefunc.cpp.o" \
"CMakeFiles/main.dir/src/calib/stereocalib.cpp.o" \
"CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o" \
"CMakeFiles/main.dir/src/stereoreconstruction.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/src/main.cpp.o
main: CMakeFiles/main.dir/src/basefunc.cpp.o
main: CMakeFiles/main.dir/src/calib/stereocalib.cpp.o
main: CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o
main: CMakeFiles/main.dir/src/stereoreconstruction.cpp.o
main: CMakeFiles/main.dir/build.make
main: /usr/local/lib/libopencv_viz.so.2.4.9
main: /usr/local/lib/libopencv_videostab.so.2.4.9
main: /usr/local/lib/libopencv_video.so.2.4.9
main: /usr/local/lib/libopencv_ts.a
main: /usr/local/lib/libopencv_superres.so.2.4.9
main: /usr/local/lib/libopencv_stitching.so.2.4.9
main: /usr/local/lib/libopencv_photo.so.2.4.9
main: /usr/local/lib/libopencv_ocl.so.2.4.9
main: /usr/local/lib/libopencv_objdetect.so.2.4.9
main: /usr/local/lib/libopencv_nonfree.so.2.4.9
main: /usr/local/lib/libopencv_ml.so.2.4.9
main: /usr/local/lib/libopencv_legacy.so.2.4.9
main: /usr/local/lib/libopencv_imgproc.so.2.4.9
main: /usr/local/lib/libopencv_highgui.so.2.4.9
main: /usr/local/lib/libopencv_gpu.so.2.4.9
main: /usr/local/lib/libopencv_flann.so.2.4.9
main: /usr/local/lib/libopencv_features2d.so.2.4.9
main: /usr/local/lib/libopencv_core.so.2.4.9
main: /usr/local/lib/libopencv_contrib.so.2.4.9
main: /usr/local/lib/libopencv_calib3d.so.2.4.9
main: /usr/lib/x86_64-linux-gnu/libGLU.so
main: /usr/lib/x86_64-linux-gnu/libGL.so
main: /usr/lib/x86_64-linux-gnu/libSM.so
main: /usr/lib/x86_64-linux-gnu/libICE.so
main: /usr/lib/x86_64-linux-gnu/libX11.so
main: /usr/lib/x86_64-linux-gnu/libXext.so
main: /usr/lib/x86_64-linux-gnu/libglut.so
main: /usr/lib/x86_64-linux-gnu/libXmu.so
main: /usr/lib/x86_64-linux-gnu/libXi.so
main: /usr/lib/x86_64-linux-gnu/libGLU.so
main: /usr/lib/x86_64-linux-gnu/libGL.so
main: /usr/lib/x86_64-linux-gnu/libSM.so
main: /usr/lib/x86_64-linux-gnu/libICE.so
main: /usr/lib/x86_64-linux-gnu/libX11.so
main: /usr/lib/x86_64-linux-gnu/libXext.so
main: /usr/local/lib/libopencv_nonfree.so.2.4.9
main: /usr/local/lib/libopencv_ocl.so.2.4.9
main: /usr/local/lib/libopencv_gpu.so.2.4.9
main: /usr/local/lib/libopencv_photo.so.2.4.9
main: /usr/local/lib/libopencv_objdetect.so.2.4.9
main: /usr/local/lib/libopencv_legacy.so.2.4.9
main: /usr/local/lib/libopencv_video.so.2.4.9
main: /usr/local/lib/libopencv_ml.so.2.4.9
main: /usr/local/lib/libopencv_calib3d.so.2.4.9
main: /usr/local/lib/libopencv_features2d.so.2.4.9
main: /usr/local/lib/libopencv_highgui.so.2.4.9
main: /usr/local/lib/libopencv_imgproc.so.2.4.9
main: /usr/local/lib/libopencv_flann.so.2.4.9
main: /usr/local/lib/libopencv_core.so.2.4.9
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main
.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/requires: CMakeFiles/main.dir/src/main.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/src/basefunc.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/src/calib/stereocalib.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/src/rectify/stereorectify.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/src/stereoreconstruction.cpp.o.requires
.PHONY : CMakeFiles/main.dir/requires

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhu/irdv_sim20160811/irdv_sim /home/zhu/irdv_sim20160811/irdv_sim /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu /home/zhu/irdv_sim20160811/irdv_sim/build_ubuntu/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

