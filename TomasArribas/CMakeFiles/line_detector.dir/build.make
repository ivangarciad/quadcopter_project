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
CMAKE_SOURCE_DIR = /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas

# Include any dependencies generated for this target.
include CMakeFiles/line_detector.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/line_detector.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/line_detector.dir/flags.make

CMakeFiles/line_detector.dir/line_detector_tomas.o: CMakeFiles/line_detector.dir/flags.make
CMakeFiles/line_detector.dir/line_detector_tomas.o: line_detector_tomas.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/line_detector.dir/line_detector_tomas.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/line_detector.dir/line_detector_tomas.o -c /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas/line_detector_tomas.cpp

CMakeFiles/line_detector.dir/line_detector_tomas.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/line_detector.dir/line_detector_tomas.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas/line_detector_tomas.cpp > CMakeFiles/line_detector.dir/line_detector_tomas.i

CMakeFiles/line_detector.dir/line_detector_tomas.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/line_detector.dir/line_detector_tomas.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas/line_detector_tomas.cpp -o CMakeFiles/line_detector.dir/line_detector_tomas.s

CMakeFiles/line_detector.dir/line_detector_tomas.o.requires:
.PHONY : CMakeFiles/line_detector.dir/line_detector_tomas.o.requires

CMakeFiles/line_detector.dir/line_detector_tomas.o.provides: CMakeFiles/line_detector.dir/line_detector_tomas.o.requires
	$(MAKE) -f CMakeFiles/line_detector.dir/build.make CMakeFiles/line_detector.dir/line_detector_tomas.o.provides.build
.PHONY : CMakeFiles/line_detector.dir/line_detector_tomas.o.provides

CMakeFiles/line_detector.dir/line_detector_tomas.o.provides.build: CMakeFiles/line_detector.dir/line_detector_tomas.o

# Object files for target line_detector
line_detector_OBJECTS = \
"CMakeFiles/line_detector.dir/line_detector_tomas.o"

# External object files for target line_detector
line_detector_EXTERNAL_OBJECTS =

line_detector: CMakeFiles/line_detector.dir/line_detector_tomas.o
line_detector: CMakeFiles/line_detector.dir/build.make
line_detector: CMakeFiles/line_detector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable line_detector"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/line_detector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/line_detector.dir/build: line_detector
.PHONY : CMakeFiles/line_detector.dir/build

CMakeFiles/line_detector.dir/requires: CMakeFiles/line_detector.dir/line_detector_tomas.o.requires
.PHONY : CMakeFiles/line_detector.dir/requires

CMakeFiles/line_detector.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/line_detector.dir/cmake_clean.cmake
.PHONY : CMakeFiles/line_detector.dir/clean

CMakeFiles/line_detector.dir/depend:
	cd /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas /home/igdaza/universidad/tfg/head/quadcopter/TomasArribas/CMakeFiles/line_detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/line_detector.dir/depend

