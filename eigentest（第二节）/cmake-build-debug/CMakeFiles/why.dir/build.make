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
include CMakeFiles/why.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/why.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/why.dir/flags.make

CMakeFiles/why.dir/why.cpp.o: CMakeFiles/why.dir/flags.make
CMakeFiles/why.dir/why.cpp.o: ../why.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/why.dir/why.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/why.dir/why.cpp.o -c /home/rmy/深蓝作业/eigentest（第二节）/why.cpp

CMakeFiles/why.dir/why.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/why.dir/why.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rmy/深蓝作业/eigentest（第二节）/why.cpp > CMakeFiles/why.dir/why.cpp.i

CMakeFiles/why.dir/why.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/why.dir/why.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rmy/深蓝作业/eigentest（第二节）/why.cpp -o CMakeFiles/why.dir/why.cpp.s

CMakeFiles/why.dir/why.cpp.o.requires:

.PHONY : CMakeFiles/why.dir/why.cpp.o.requires

CMakeFiles/why.dir/why.cpp.o.provides: CMakeFiles/why.dir/why.cpp.o.requires
	$(MAKE) -f CMakeFiles/why.dir/build.make CMakeFiles/why.dir/why.cpp.o.provides.build
.PHONY : CMakeFiles/why.dir/why.cpp.o.provides

CMakeFiles/why.dir/why.cpp.o.provides.build: CMakeFiles/why.dir/why.cpp.o


# Object files for target why
why_OBJECTS = \
"CMakeFiles/why.dir/why.cpp.o"

# External object files for target why
why_EXTERNAL_OBJECTS =

why: CMakeFiles/why.dir/why.cpp.o
why: CMakeFiles/why.dir/build.make
why: CMakeFiles/why.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable why"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/why.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/why.dir/build: why

.PHONY : CMakeFiles/why.dir/build

CMakeFiles/why.dir/requires: CMakeFiles/why.dir/why.cpp.o.requires

.PHONY : CMakeFiles/why.dir/requires

CMakeFiles/why.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/why.dir/cmake_clean.cmake
.PHONY : CMakeFiles/why.dir/clean

CMakeFiles/why.dir/depend:
	cd /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rmy/深蓝作业/eigentest（第二节） /home/rmy/深蓝作业/eigentest（第二节） /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug /home/rmy/深蓝作业/eigentest（第二节）/cmake-build-debug/CMakeFiles/why.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/why.dir/depend

