# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /opt/clion-2020.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2020.3.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mitom/Music/clion_workspace/BMEA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/BMEA.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BMEA.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BMEA.dir/flags.make

CMakeFiles/BMEA.dir/main.cpp.o: CMakeFiles/BMEA.dir/flags.make
CMakeFiles/BMEA.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BMEA.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BMEA.dir/main.cpp.o -c /home/mitom/Music/clion_workspace/BMEA/main.cpp

CMakeFiles/BMEA.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BMEA.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mitom/Music/clion_workspace/BMEA/main.cpp > CMakeFiles/BMEA.dir/main.cpp.i

CMakeFiles/BMEA.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BMEA.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mitom/Music/clion_workspace/BMEA/main.cpp -o CMakeFiles/BMEA.dir/main.cpp.s

# Object files for target BMEA
BMEA_OBJECTS = \
"CMakeFiles/BMEA.dir/main.cpp.o"

# External object files for target BMEA
BMEA_EXTERNAL_OBJECTS =

BMEA: CMakeFiles/BMEA.dir/main.cpp.o
BMEA: CMakeFiles/BMEA.dir/build.make
BMEA: util/libutil.a
BMEA: /usr/local/lib/libopencv_gapi.so.4.5.3
BMEA: /usr/local/lib/libopencv_highgui.so.4.5.3
BMEA: /usr/local/lib/libopencv_ml.so.4.5.3
BMEA: /usr/local/lib/libopencv_objdetect.so.4.5.3
BMEA: /usr/local/lib/libopencv_photo.so.4.5.3
BMEA: /usr/local/lib/libopencv_stitching.so.4.5.3
BMEA: /usr/local/lib/libopencv_video.so.4.5.3
BMEA: /usr/local/lib/libopencv_videoio.so.4.5.3
BMEA: /usr/lib/x86_64-linux-gnu/libpython3.8.so
BMEA: /usr/local/lib/libopencv_dnn.so.4.5.3
BMEA: /usr/local/lib/libopencv_imgcodecs.so.4.5.3
BMEA: /usr/local/lib/libopencv_calib3d.so.4.5.3
BMEA: /usr/local/lib/libopencv_features2d.so.4.5.3
BMEA: /usr/local/lib/libopencv_flann.so.4.5.3
BMEA: /usr/local/lib/libopencv_imgproc.so.4.5.3
BMEA: /usr/local/lib/libopencv_core.so.4.5.3
BMEA: CMakeFiles/BMEA.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable BMEA"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BMEA.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BMEA.dir/build: BMEA

.PHONY : CMakeFiles/BMEA.dir/build

CMakeFiles/BMEA.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BMEA.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BMEA.dir/clean

CMakeFiles/BMEA.dir/depend:
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mitom/Music/clion_workspace/BMEA /home/mitom/Music/clion_workspace/BMEA /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/CMakeFiles/BMEA.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BMEA.dir/depend

