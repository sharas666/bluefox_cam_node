# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/josh/uni/project/bluefox_cam_node/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/josh/uni/project/bluefox_cam_node/catkin_ws/build

# Include any dependencies generated for this target.
include bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/depend.make

# Include the progress variables for this target.
include bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/progress.make

# Include the compile flags for this target's objects.
include bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/flags.make

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o: bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/flags.make
bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o: /home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/src/mvStereoVision/src/Camera.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o"
	cd /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o -c /home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/src/mvStereoVision/src/Camera.cpp

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.i"
	cd /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/src/mvStereoVision/src/Camera.cpp > CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.i

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.s"
	cd /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/src/mvStereoVision/src/Camera.cpp -o CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.s

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.requires:
.PHONY : bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.requires

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.provides: bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.requires
	$(MAKE) -f bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/build.make bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.provides.build
.PHONY : bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.provides

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.provides.build: bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o

# Object files for target mvStereoVisionCamera2
mvStereoVisionCamera2_OBJECTS = \
"CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o"

# External object files for target mvStereoVisionCamera2
mvStereoVisionCamera2_EXTERNAL_OBJECTS =

/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/libmvStereoVisionCamera2.so: bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o
/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/libmvStereoVisionCamera2.so: bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/build.make
/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/libmvStereoVisionCamera2.so: bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/libmvStereoVisionCamera2.so"
	cd /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mvStereoVisionCamera2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/build: /home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/libmvStereoVisionCamera2.so
.PHONY : bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/build

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/requires: bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/src/mvStereoVision/src/Camera.cpp.o.requires
.PHONY : bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/requires

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/clean:
	cd /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node && $(CMAKE_COMMAND) -P CMakeFiles/mvStereoVisionCamera2.dir/cmake_clean.cmake
.PHONY : bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/clean

bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/depend:
	cd /home/josh/uni/project/bluefox_cam_node/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/josh/uni/project/bluefox_cam_node/catkin_ws/src /home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node /home/josh/uni/project/bluefox_cam_node/catkin_ws/build /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node /home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : bluefox_cam_node/CMakeFiles/mvStereoVisionCamera2.dir/depend

