# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/alessandro/prog_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alessandro/prog_ws/build

# Utility rule file for _run_tests_explore_lite.

# Include the progress variables for this target.
include packages/explore/CMakeFiles/_run_tests_explore_lite.dir/progress.make

_run_tests_explore_lite: packages/explore/CMakeFiles/_run_tests_explore_lite.dir/build.make

.PHONY : _run_tests_explore_lite

# Rule to build all files generated by this target.
packages/explore/CMakeFiles/_run_tests_explore_lite.dir/build: _run_tests_explore_lite

.PHONY : packages/explore/CMakeFiles/_run_tests_explore_lite.dir/build

packages/explore/CMakeFiles/_run_tests_explore_lite.dir/clean:
	cd /home/alessandro/prog_ws/build/packages/explore && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_explore_lite.dir/cmake_clean.cmake
.PHONY : packages/explore/CMakeFiles/_run_tests_explore_lite.dir/clean

packages/explore/CMakeFiles/_run_tests_explore_lite.dir/depend:
	cd /home/alessandro/prog_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alessandro/prog_ws/src /home/alessandro/prog_ws/src/packages/explore /home/alessandro/prog_ws/build /home/alessandro/prog_ws/build/packages/explore /home/alessandro/prog_ws/build/packages/explore/CMakeFiles/_run_tests_explore_lite.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : packages/explore/CMakeFiles/_run_tests_explore_lite.dir/depend

