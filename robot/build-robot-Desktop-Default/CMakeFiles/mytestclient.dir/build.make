# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/cmake-3.12.3-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /usr/cmake-3.12.3-Linux-x86_64/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kaanh/Desktop/robot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaanh/Desktop/build-robot-Desktop-Default

# Include any dependencies generated for this target.
include CMakeFiles/mytestclient.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mytestclient.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mytestclient.dir/flags.make

CMakeFiles/mytestclient.dir/src/mytestclient.cpp.o: CMakeFiles/mytestclient.dir/flags.make
CMakeFiles/mytestclient.dir/src/mytestclient.cpp.o: /home/kaanh/Desktop/robot/src/mytestclient.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaanh/Desktop/build-robot-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mytestclient.dir/src/mytestclient.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mytestclient.dir/src/mytestclient.cpp.o -c /home/kaanh/Desktop/robot/src/mytestclient.cpp

CMakeFiles/mytestclient.dir/src/mytestclient.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mytestclient.dir/src/mytestclient.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaanh/Desktop/robot/src/mytestclient.cpp > CMakeFiles/mytestclient.dir/src/mytestclient.cpp.i

CMakeFiles/mytestclient.dir/src/mytestclient.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mytestclient.dir/src/mytestclient.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaanh/Desktop/robot/src/mytestclient.cpp -o CMakeFiles/mytestclient.dir/src/mytestclient.cpp.s

# Object files for target mytestclient
mytestclient_OBJECTS = \
"CMakeFiles/mytestclient.dir/src/mytestclient.cpp.o"

# External object files for target mytestclient
mytestclient_EXTERNAL_OBJECTS =

mytestclient: CMakeFiles/mytestclient.dir/src/mytestclient.cpp.o
mytestclient: CMakeFiles/mytestclient.dir/build.make
mytestclient: CMakeFiles/mytestclient.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaanh/Desktop/build-robot-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mytestclient"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mytestclient.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mytestclient.dir/build: mytestclient

.PHONY : CMakeFiles/mytestclient.dir/build

CMakeFiles/mytestclient.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mytestclient.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mytestclient.dir/clean

CMakeFiles/mytestclient.dir/depend:
	cd /home/kaanh/Desktop/build-robot-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaanh/Desktop/robot /home/kaanh/Desktop/robot /home/kaanh/Desktop/build-robot-Desktop-Default /home/kaanh/Desktop/build-robot-Desktop-Default /home/kaanh/Desktop/build-robot-Desktop-Default/CMakeFiles/mytestclient.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mytestclient.dir/depend

