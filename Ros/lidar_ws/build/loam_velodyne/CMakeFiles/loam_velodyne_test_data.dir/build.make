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
CMAKE_SOURCE_DIR = /home/rip-acer-vn7-591g-1/lidar_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rip-acer-vn7-591g-1/lidar_ws/build

# Utility rule file for loam_velodyne_test_data.

# Include the progress variables for this target.
include loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/progress.make

loam_velodyne/CMakeFiles/loam_velodyne_test_data:
	cd /home/rip-acer-vn7-591g-1/lidar_ws/build/loam_velodyne && /usr/bin/cmake -E tar -xzf velodyne_loam_test_data.tar.gz

loam_velodyne_test_data: loam_velodyne/CMakeFiles/loam_velodyne_test_data
loam_velodyne_test_data: loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/build.make

.PHONY : loam_velodyne_test_data

# Rule to build all files generated by this target.
loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/build: loam_velodyne_test_data

.PHONY : loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/build

loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/clean:
	cd /home/rip-acer-vn7-591g-1/lidar_ws/build/loam_velodyne && $(CMAKE_COMMAND) -P CMakeFiles/loam_velodyne_test_data.dir/cmake_clean.cmake
.PHONY : loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/clean

loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/depend:
	cd /home/rip-acer-vn7-591g-1/lidar_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rip-acer-vn7-591g-1/lidar_ws/src /home/rip-acer-vn7-591g-1/lidar_ws/src/loam_velodyne /home/rip-acer-vn7-591g-1/lidar_ws/build /home/rip-acer-vn7-591g-1/lidar_ws/build/loam_velodyne /home/rip-acer-vn7-591g-1/lidar_ws/build/loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : loam_velodyne/CMakeFiles/loam_velodyne_test_data.dir/depend

