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
CMAKE_SOURCE_DIR = /home/tb/gazebo_road_generation/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tb/gazebo_road_generation/ros_ws/build

# Include any dependencies generated for this target.
include track_annotation/CMakeFiles/annotate_bag.dir/depend.make

# Include the progress variables for this target.
include track_annotation/CMakeFiles/annotate_bag.dir/progress.make

# Include the compile flags for this target's objects.
include track_annotation/CMakeFiles/annotate_bag.dir/flags.make

track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o: track_annotation/CMakeFiles/annotate_bag.dir/flags.make
track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o: /home/tb/gazebo_road_generation/ros_ws/src/track_annotation/src/annotate_bag.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tb/gazebo_road_generation/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o"
	cd /home/tb/gazebo_road_generation/ros_ws/build/track_annotation && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o -c /home/tb/gazebo_road_generation/ros_ws/src/track_annotation/src/annotate_bag.cpp

track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.i"
	cd /home/tb/gazebo_road_generation/ros_ws/build/track_annotation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tb/gazebo_road_generation/ros_ws/src/track_annotation/src/annotate_bag.cpp > CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.i

track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.s"
	cd /home/tb/gazebo_road_generation/ros_ws/build/track_annotation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tb/gazebo_road_generation/ros_ws/src/track_annotation/src/annotate_bag.cpp -o CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.s

track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.requires:

.PHONY : track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.requires

track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.provides: track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.requires
	$(MAKE) -f track_annotation/CMakeFiles/annotate_bag.dir/build.make track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.provides.build
.PHONY : track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.provides

track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.provides.build: track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o


# Object files for target annotate_bag
annotate_bag_OBJECTS = \
"CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o"

# External object files for target annotate_bag
annotate_bag_EXTERNAL_OBJECTS =

/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: track_annotation/CMakeFiles/annotate_bag.dir/build.make
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libcv_bridge.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libimage_transport.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libmessage_filters.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libclass_loader.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/libPocoFoundation.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libdl.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libroslib.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/librospack.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libroscpp.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/librosconsole.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/librostime.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /opt/ros/melodic/lib/libcpp_common.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag: track_annotation/CMakeFiles/annotate_bag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tb/gazebo_road_generation/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag"
	cd /home/tb/gazebo_road_generation/ros_ws/build/track_annotation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/annotate_bag.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
track_annotation/CMakeFiles/annotate_bag.dir/build: /home/tb/gazebo_road_generation/ros_ws/devel/lib/track_annotation/annotate_bag

.PHONY : track_annotation/CMakeFiles/annotate_bag.dir/build

track_annotation/CMakeFiles/annotate_bag.dir/requires: track_annotation/CMakeFiles/annotate_bag.dir/src/annotate_bag.cpp.o.requires

.PHONY : track_annotation/CMakeFiles/annotate_bag.dir/requires

track_annotation/CMakeFiles/annotate_bag.dir/clean:
	cd /home/tb/gazebo_road_generation/ros_ws/build/track_annotation && $(CMAKE_COMMAND) -P CMakeFiles/annotate_bag.dir/cmake_clean.cmake
.PHONY : track_annotation/CMakeFiles/annotate_bag.dir/clean

track_annotation/CMakeFiles/annotate_bag.dir/depend:
	cd /home/tb/gazebo_road_generation/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tb/gazebo_road_generation/ros_ws/src /home/tb/gazebo_road_generation/ros_ws/src/track_annotation /home/tb/gazebo_road_generation/ros_ws/build /home/tb/gazebo_road_generation/ros_ws/build/track_annotation /home/tb/gazebo_road_generation/ros_ws/build/track_annotation/CMakeFiles/annotate_bag.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : track_annotation/CMakeFiles/annotate_bag.dir/depend
