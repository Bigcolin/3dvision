# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.26.3/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.26.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kyan/Projects/3dpcl/pcl0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kyan/Projects/3dpcl/pcl0/build

# Include any dependencies generated for this target.
include CMakeFiles/ndt_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ndt_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ndt_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ndt_test.dir/flags.make

CMakeFiles/ndt_test.dir/ndt.cpp.o: CMakeFiles/ndt_test.dir/flags.make
CMakeFiles/ndt_test.dir/ndt.cpp.o: /Users/kyan/Projects/3dpcl/pcl0/ndt.cpp
CMakeFiles/ndt_test.dir/ndt.cpp.o: CMakeFiles/ndt_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kyan/Projects/3dpcl/pcl0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ndt_test.dir/ndt.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ndt_test.dir/ndt.cpp.o -MF CMakeFiles/ndt_test.dir/ndt.cpp.o.d -o CMakeFiles/ndt_test.dir/ndt.cpp.o -c /Users/kyan/Projects/3dpcl/pcl0/ndt.cpp

CMakeFiles/ndt_test.dir/ndt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ndt_test.dir/ndt.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kyan/Projects/3dpcl/pcl0/ndt.cpp > CMakeFiles/ndt_test.dir/ndt.cpp.i

CMakeFiles/ndt_test.dir/ndt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ndt_test.dir/ndt.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kyan/Projects/3dpcl/pcl0/ndt.cpp -o CMakeFiles/ndt_test.dir/ndt.cpp.s

# Object files for target ndt_test
ndt_test_OBJECTS = \
"CMakeFiles/ndt_test.dir/ndt.cpp.o"

# External object files for target ndt_test
ndt_test_EXTERNAL_OBJECTS =

ndt_test: CMakeFiles/ndt_test.dir/ndt.cpp.o
ndt_test: CMakeFiles/ndt_test.dir/build.make
ndt_test: CMakeFiles/ndt_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/kyan/Projects/3dpcl/pcl0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ndt_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ndt_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ndt_test.dir/build: ndt_test
.PHONY : CMakeFiles/ndt_test.dir/build

CMakeFiles/ndt_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ndt_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ndt_test.dir/clean

CMakeFiles/ndt_test.dir/depend:
	cd /Users/kyan/Projects/3dpcl/pcl0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kyan/Projects/3dpcl/pcl0 /Users/kyan/Projects/3dpcl/pcl0 /Users/kyan/Projects/3dpcl/pcl0/build /Users/kyan/Projects/3dpcl/pcl0/build /Users/kyan/Projects/3dpcl/pcl0/build/CMakeFiles/ndt_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ndt_test.dir/depend

