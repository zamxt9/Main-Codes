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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_workspace/src/serial

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_workspace/src/serial

# Include any dependencies generated for this target.
include tests/CMakeFiles/serial-test.dir/depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/serial-test.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/serial-test.dir/flags.make

tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o: tests/CMakeFiles/serial-test.dir/flags.make
tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o: tests/unix_serial_tests.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/catkin_workspace/src/serial/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o"
	cd /home/ubuntu/catkin_workspace/src/serial/tests && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/serial-test.dir/unix_serial_tests.cc.o -c /home/ubuntu/catkin_workspace/src/serial/tests/unix_serial_tests.cc

tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial-test.dir/unix_serial_tests.cc.i"
	cd /home/ubuntu/catkin_workspace/src/serial/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/catkin_workspace/src/serial/tests/unix_serial_tests.cc > CMakeFiles/serial-test.dir/unix_serial_tests.cc.i

tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial-test.dir/unix_serial_tests.cc.s"
	cd /home/ubuntu/catkin_workspace/src/serial/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/catkin_workspace/src/serial/tests/unix_serial_tests.cc -o CMakeFiles/serial-test.dir/unix_serial_tests.cc.s

tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires:
.PHONY : tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires

tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides: tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires
	$(MAKE) -f tests/CMakeFiles/serial-test.dir/build.make tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides.build
.PHONY : tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides

tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides.build: tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o

# Object files for target serial-test
serial__test_OBJECTS = \
"CMakeFiles/serial-test.dir/unix_serial_tests.cc.o"

# External object files for target serial-test
serial__test_EXTERNAL_OBJECTS =

devel/lib/serial/serial-test: tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o
devel/lib/serial/serial-test: gtest/libgtest.so
devel/lib/serial/serial-test: devel/lib/libserial.so
devel/lib/serial/serial-test: tests/CMakeFiles/serial-test.dir/build.make
devel/lib/serial/serial-test: tests/CMakeFiles/serial-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../devel/lib/serial/serial-test"
	cd /home/ubuntu/catkin_workspace/src/serial/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/serial-test.dir/build: devel/lib/serial/serial-test
.PHONY : tests/CMakeFiles/serial-test.dir/build

tests/CMakeFiles/serial-test.dir/requires: tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires
.PHONY : tests/CMakeFiles/serial-test.dir/requires

tests/CMakeFiles/serial-test.dir/clean:
	cd /home/ubuntu/catkin_workspace/src/serial/tests && $(CMAKE_COMMAND) -P CMakeFiles/serial-test.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/serial-test.dir/clean

tests/CMakeFiles/serial-test.dir/depend:
	cd /home/ubuntu/catkin_workspace/src/serial && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_workspace/src/serial /home/ubuntu/catkin_workspace/src/serial/tests /home/ubuntu/catkin_workspace/src/serial /home/ubuntu/catkin_workspace/src/serial/tests /home/ubuntu/catkin_workspace/src/serial/tests/CMakeFiles/serial-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/serial-test.dir/depend

