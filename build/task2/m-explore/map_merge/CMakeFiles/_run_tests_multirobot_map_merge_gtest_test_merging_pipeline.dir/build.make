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

# Utility rule file for _run_tests_multirobot_map_merge_gtest_test_merging_pipeline.

# Include the progress variables for this target.
include task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/progress.make

task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline:
	cd /home/alessandro/prog_ws/build/task2/m-explore/map_merge && ../../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/alessandro/prog_ws/build/test_results/multirobot_map_merge/gtest-test_merging_pipeline.xml "/home/alessandro/prog_ws/devel/lib/multirobot_map_merge/test_merging_pipeline --gtest_output=xml:/home/alessandro/prog_ws/build/test_results/multirobot_map_merge/gtest-test_merging_pipeline.xml"

_run_tests_multirobot_map_merge_gtest_test_merging_pipeline: task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline
_run_tests_multirobot_map_merge_gtest_test_merging_pipeline: task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/build.make

.PHONY : _run_tests_multirobot_map_merge_gtest_test_merging_pipeline

# Rule to build all files generated by this target.
task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/build: _run_tests_multirobot_map_merge_gtest_test_merging_pipeline

.PHONY : task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/build

task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/clean:
	cd /home/alessandro/prog_ws/build/task2/m-explore/map_merge && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/cmake_clean.cmake
.PHONY : task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/clean

task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/depend:
	cd /home/alessandro/prog_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alessandro/prog_ws/src /home/alessandro/prog_ws/src/task2/m-explore/map_merge /home/alessandro/prog_ws/build /home/alessandro/prog_ws/build/task2/m-explore/map_merge /home/alessandro/prog_ws/build/task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : task2/m-explore/map_merge/CMakeFiles/_run_tests_multirobot_map_merge_gtest_test_merging_pipeline.dir/depend

