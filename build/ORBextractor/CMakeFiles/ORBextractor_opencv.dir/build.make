# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/hao/motion_tracking_design/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hao/motion_tracking_design/build

# Include any dependencies generated for this target.
include ORBextractor/CMakeFiles/ORBextractor_opencv.dir/depend.make

# Include the progress variables for this target.
include ORBextractor/CMakeFiles/ORBextractor_opencv.dir/progress.make

# Include the compile flags for this target's objects.
include ORBextractor/CMakeFiles/ORBextractor_opencv.dir/flags.make

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o: ORBextractor/CMakeFiles/ORBextractor_opencv.dir/flags.make
ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o: /home/hao/motion_tracking_design/src/ORBextractor/src/ORBextractor_opencv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hao/motion_tracking_design/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o"
	cd /home/hao/motion_tracking_design/build/ORBextractor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o -c /home/hao/motion_tracking_design/src/ORBextractor/src/ORBextractor_opencv.cpp

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.i"
	cd /home/hao/motion_tracking_design/build/ORBextractor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hao/motion_tracking_design/src/ORBextractor/src/ORBextractor_opencv.cpp > CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.i

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.s"
	cd /home/hao/motion_tracking_design/build/ORBextractor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hao/motion_tracking_design/src/ORBextractor/src/ORBextractor_opencv.cpp -o CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.s

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.requires:

.PHONY : ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.requires

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.provides: ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.requires
	$(MAKE) -f ORBextractor/CMakeFiles/ORBextractor_opencv.dir/build.make ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.provides.build
.PHONY : ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.provides

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.provides.build: ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o


# Object files for target ORBextractor_opencv
ORBextractor_opencv_OBJECTS = \
"CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o"

# External object files for target ORBextractor_opencv
ORBextractor_opencv_EXTERNAL_OBJECTS =

/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: ORBextractor/CMakeFiles/ORBextractor_opencv.dir/build.make
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libcv_bridge.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libimage_transport.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libclass_loader.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/libPocoFoundation.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libdl.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libroslib.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/librospack.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libmessage_filters.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libroscpp.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/librosconsole.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/librostime.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /opt/ros/melodic/lib/libcpp_common.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/local/lib/librealsense2.so.2.48.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv: ORBextractor/CMakeFiles/ORBextractor_opencv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hao/motion_tracking_design/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv"
	cd /home/hao/motion_tracking_design/build/ORBextractor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ORBextractor_opencv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ORBextractor/CMakeFiles/ORBextractor_opencv.dir/build: /home/hao/motion_tracking_design/devel/lib/ORBextractor/ORBextractor_opencv

.PHONY : ORBextractor/CMakeFiles/ORBextractor_opencv.dir/build

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/requires: ORBextractor/CMakeFiles/ORBextractor_opencv.dir/src/ORBextractor_opencv.cpp.o.requires

.PHONY : ORBextractor/CMakeFiles/ORBextractor_opencv.dir/requires

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/clean:
	cd /home/hao/motion_tracking_design/build/ORBextractor && $(CMAKE_COMMAND) -P CMakeFiles/ORBextractor_opencv.dir/cmake_clean.cmake
.PHONY : ORBextractor/CMakeFiles/ORBextractor_opencv.dir/clean

ORBextractor/CMakeFiles/ORBextractor_opencv.dir/depend:
	cd /home/hao/motion_tracking_design/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hao/motion_tracking_design/src /home/hao/motion_tracking_design/src/ORBextractor /home/hao/motion_tracking_design/build /home/hao/motion_tracking_design/build/ORBextractor /home/hao/motion_tracking_design/build/ORBextractor/CMakeFiles/ORBextractor_opencv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ORBextractor/CMakeFiles/ORBextractor_opencv.dir/depend

