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
include util/CMakeFiles/util.dir/depend.make

# Include the progress variables for this target.
include util/CMakeFiles/util.dir/progress.make

# Include the compile flags for this target's objects.
include util/CMakeFiles/util.dir/flags.make

util/CMakeFiles/util.dir/string_util.cpp.o: util/CMakeFiles/util.dir/flags.make
util/CMakeFiles/util.dir/string_util.cpp.o: ../util/string_util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object util/CMakeFiles/util.dir/string_util.cpp.o"
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/util.dir/string_util.cpp.o -c /home/mitom/Music/clion_workspace/BMEA/util/string_util.cpp

util/CMakeFiles/util.dir/string_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/util.dir/string_util.cpp.i"
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mitom/Music/clion_workspace/BMEA/util/string_util.cpp > CMakeFiles/util.dir/string_util.cpp.i

util/CMakeFiles/util.dir/string_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/util.dir/string_util.cpp.s"
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mitom/Music/clion_workspace/BMEA/util/string_util.cpp -o CMakeFiles/util.dir/string_util.cpp.s

util/CMakeFiles/util.dir/math.cpp.o: util/CMakeFiles/util.dir/flags.make
util/CMakeFiles/util.dir/math.cpp.o: ../util/math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object util/CMakeFiles/util.dir/math.cpp.o"
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/util.dir/math.cpp.o -c /home/mitom/Music/clion_workspace/BMEA/util/math.cpp

util/CMakeFiles/util.dir/math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/util.dir/math.cpp.i"
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mitom/Music/clion_workspace/BMEA/util/math.cpp > CMakeFiles/util.dir/math.cpp.i

util/CMakeFiles/util.dir/math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/util.dir/math.cpp.s"
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mitom/Music/clion_workspace/BMEA/util/math.cpp -o CMakeFiles/util.dir/math.cpp.s

# Object files for target util
util_OBJECTS = \
"CMakeFiles/util.dir/string_util.cpp.o" \
"CMakeFiles/util.dir/math.cpp.o"

# External object files for target util
util_EXTERNAL_OBJECTS =

util/libutil.a: util/CMakeFiles/util.dir/string_util.cpp.o
util/libutil.a: util/CMakeFiles/util.dir/math.cpp.o
util/libutil.a: util/CMakeFiles/util.dir/build.make
util/libutil.a: util/CMakeFiles/util.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libutil.a"
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && $(CMAKE_COMMAND) -P CMakeFiles/util.dir/cmake_clean_target.cmake
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/util.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
util/CMakeFiles/util.dir/build: util/libutil.a

.PHONY : util/CMakeFiles/util.dir/build

util/CMakeFiles/util.dir/clean:
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util && $(CMAKE_COMMAND) -P CMakeFiles/util.dir/cmake_clean.cmake
.PHONY : util/CMakeFiles/util.dir/clean

util/CMakeFiles/util.dir/depend:
	cd /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mitom/Music/clion_workspace/BMEA /home/mitom/Music/clion_workspace/BMEA/util /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util /home/mitom/Music/clion_workspace/BMEA/cmake-build-debug/util/CMakeFiles/util.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : util/CMakeFiles/util.dir/depend
