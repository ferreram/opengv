# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mferrera/libs/opengv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mferrera/libs/opengv/build

# Include any dependencies generated for this target.
include CMakeFiles/test_Sturm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_Sturm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_Sturm.dir/flags.make

CMakeFiles/test_Sturm.dir/test/test_Sturm.o: CMakeFiles/test_Sturm.dir/flags.make
CMakeFiles/test_Sturm.dir/test/test_Sturm.o: ../test/test_Sturm.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/mferrera/libs/opengv/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_Sturm.dir/test/test_Sturm.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_Sturm.dir/test/test_Sturm.o -c /home/mferrera/libs/opengv/test/test_Sturm.cpp

CMakeFiles/test_Sturm.dir/test/test_Sturm.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_Sturm.dir/test/test_Sturm.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/mferrera/libs/opengv/test/test_Sturm.cpp > CMakeFiles/test_Sturm.dir/test/test_Sturm.i

CMakeFiles/test_Sturm.dir/test/test_Sturm.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_Sturm.dir/test/test_Sturm.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/mferrera/libs/opengv/test/test_Sturm.cpp -o CMakeFiles/test_Sturm.dir/test/test_Sturm.s

CMakeFiles/test_Sturm.dir/test/test_Sturm.o.requires:
.PHONY : CMakeFiles/test_Sturm.dir/test/test_Sturm.o.requires

CMakeFiles/test_Sturm.dir/test/test_Sturm.o.provides: CMakeFiles/test_Sturm.dir/test/test_Sturm.o.requires
	$(MAKE) -f CMakeFiles/test_Sturm.dir/build.make CMakeFiles/test_Sturm.dir/test/test_Sturm.o.provides.build
.PHONY : CMakeFiles/test_Sturm.dir/test/test_Sturm.o.provides

CMakeFiles/test_Sturm.dir/test/test_Sturm.o.provides.build: CMakeFiles/test_Sturm.dir/test/test_Sturm.o

# Object files for target test_Sturm
test_Sturm_OBJECTS = \
"CMakeFiles/test_Sturm.dir/test/test_Sturm.o"

# External object files for target test_Sturm
test_Sturm_EXTERNAL_OBJECTS =

bin/test_Sturm: CMakeFiles/test_Sturm.dir/test/test_Sturm.o
bin/test_Sturm: CMakeFiles/test_Sturm.dir/build.make
bin/test_Sturm: lib/libopengv.a
bin/test_Sturm: lib/librandom_generators.a
bin/test_Sturm: lib/libopengv.a
bin/test_Sturm: CMakeFiles/test_Sturm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/test_Sturm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_Sturm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_Sturm.dir/build: bin/test_Sturm
.PHONY : CMakeFiles/test_Sturm.dir/build

CMakeFiles/test_Sturm.dir/requires: CMakeFiles/test_Sturm.dir/test/test_Sturm.o.requires
.PHONY : CMakeFiles/test_Sturm.dir/requires

CMakeFiles/test_Sturm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_Sturm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_Sturm.dir/clean

CMakeFiles/test_Sturm.dir/depend:
	cd /home/mferrera/libs/opengv/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mferrera/libs/opengv /home/mferrera/libs/opengv /home/mferrera/libs/opengv/build /home/mferrera/libs/opengv/build /home/mferrera/libs/opengv/build/CMakeFiles/test_Sturm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_Sturm.dir/depend

