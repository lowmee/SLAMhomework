# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /home/rmy/clion-2017.3.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/rmy/clion-2017.3.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rmy/深蓝作业/eigentest（第二节）

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/decomposition.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/decomposition.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/decomposition.dir/flags.make

CMakeFiles/decomposition.dir/decomposition.cpp.o: CMakeFiles/decomposition.dir/flags.make
CMakeFiles/decomposition.dir/decomposition.cpp.o: ../decomposition.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/decomposition.dir/decomposition.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/decomposition.dir/decomposition.cpp.o -c /home/rmy/深蓝作业/eigentest（第二节）/decomposition.cpp

CMakeFiles/decomposition.dir/decomposition.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/decomposition.dir/decomposition.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmy/深蓝作业/eigentest（第二节）/decomposition.cpp > CMakeFiles/decomposition.dir/decomposition.cpp.i

CMakeFiles/decomposition.dir/decomposition.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/decomposition.dir/decomposition.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmy/深蓝作业/eigentest（第二节）/decomposition.cpp -o CMakeFiles/decomposition.dir/decomposition.cpp.s

CMakeFiles/decomposition.dir/decomposition.cpp.o.requires:

.PHONY : CMakeFiles/decomposition.dir/decomposition.cpp.o.requires

CMakeFiles/decomposition.dir/decomposition.cpp.o.provides: CMakeFiles/decomposition.dir/decomposition.cpp.o.requires
	$(MAKE) -f CMakeFiles/decomposition.dir/build.make CMakeFiles/decomposition.dir/decomposition.cpp.o.provides.build
.PHONY : CMakeFiles/decomposition.dir/decomposition.cpp.o.provides

CMakeFiles/decomposition.dir/decomposition.cpp.o.provides.build: CMakeFiles/decomposition.dir/decomposition.cpp.o


# Object files for target decomposition
decomposition_OBJECTS = \
"CMakeFiles/decomposition.dir/decomposition.cpp.o"

# External object files for target decomposition
decomposition_EXTERNAL_OBJECTS =

decomposition: CMakeFiles/decomposition.dir/decomposition.cpp.o
decomposition: CMakeFiles/decomposition.dir/build.make
decomposition: CMakeFiles/decomposition.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable decomposition"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/decomposition.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/decomposition.dir/build: decomposition

.PHONY : CMakeFiles/decomposition.dir/build

CMakeFiles/decomposition.dir/requires: CMakeFiles/decomposition.dir/decomposition.cpp.o.requires

.PHONY : CMakeFiles/decomposition.dir/requires

CMakeFiles/decomposition.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/decomposition.dir/cmake_clean.cmake
.PHONY : CMakeFiles/decomposition.dir/clean

CMakeFiles/decomposition.dir/depend:
	cd /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rmy/深蓝作业/eigentest（第二节） /home/rmy/深蓝作业/eigentest（第二节） /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug/CMakeFiles/decomposition.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/decomposition.dir/depend

