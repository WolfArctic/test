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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/idriver/study/LRegression

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/idriver/study/LRegression

# Include any dependencies generated for this target.
include CMakeFiles/LRegression.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LRegression.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LRegression.dir/flags.make

CMakeFiles/LRegression.dir/src/main.cpp.o: CMakeFiles/LRegression.dir/flags.make
CMakeFiles/LRegression.dir/src/main.cpp.o: src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/idriver/study/LRegression/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LRegression.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LRegression.dir/src/main.cpp.o -c /home/idriver/study/LRegression/src/main.cpp

CMakeFiles/LRegression.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LRegression.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/idriver/study/LRegression/src/main.cpp > CMakeFiles/LRegression.dir/src/main.cpp.i

CMakeFiles/LRegression.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LRegression.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/idriver/study/LRegression/src/main.cpp -o CMakeFiles/LRegression.dir/src/main.cpp.s

CMakeFiles/LRegression.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/LRegression.dir/src/main.cpp.o.requires

CMakeFiles/LRegression.dir/src/main.cpp.o.provides: CMakeFiles/LRegression.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/LRegression.dir/build.make CMakeFiles/LRegression.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/LRegression.dir/src/main.cpp.o.provides

CMakeFiles/LRegression.dir/src/main.cpp.o.provides.build: CMakeFiles/LRegression.dir/src/main.cpp.o

CMakeFiles/LRegression.dir/src/LRegression.cpp.o: CMakeFiles/LRegression.dir/flags.make
CMakeFiles/LRegression.dir/src/LRegression.cpp.o: src/LRegression.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/idriver/study/LRegression/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LRegression.dir/src/LRegression.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LRegression.dir/src/LRegression.cpp.o -c /home/idriver/study/LRegression/src/LRegression.cpp

CMakeFiles/LRegression.dir/src/LRegression.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LRegression.dir/src/LRegression.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/idriver/study/LRegression/src/LRegression.cpp > CMakeFiles/LRegression.dir/src/LRegression.cpp.i

CMakeFiles/LRegression.dir/src/LRegression.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LRegression.dir/src/LRegression.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/idriver/study/LRegression/src/LRegression.cpp -o CMakeFiles/LRegression.dir/src/LRegression.cpp.s

CMakeFiles/LRegression.dir/src/LRegression.cpp.o.requires:
.PHONY : CMakeFiles/LRegression.dir/src/LRegression.cpp.o.requires

CMakeFiles/LRegression.dir/src/LRegression.cpp.o.provides: CMakeFiles/LRegression.dir/src/LRegression.cpp.o.requires
	$(MAKE) -f CMakeFiles/LRegression.dir/build.make CMakeFiles/LRegression.dir/src/LRegression.cpp.o.provides.build
.PHONY : CMakeFiles/LRegression.dir/src/LRegression.cpp.o.provides

CMakeFiles/LRegression.dir/src/LRegression.cpp.o.provides.build: CMakeFiles/LRegression.dir/src/LRegression.cpp.o

# Object files for target LRegression
LRegression_OBJECTS = \
"CMakeFiles/LRegression.dir/src/main.cpp.o" \
"CMakeFiles/LRegression.dir/src/LRegression.cpp.o"

# External object files for target LRegression
LRegression_EXTERNAL_OBJECTS =

LRegression: CMakeFiles/LRegression.dir/src/main.cpp.o
LRegression: CMakeFiles/LRegression.dir/src/LRegression.cpp.o
LRegression: CMakeFiles/LRegression.dir/build.make
LRegression: CMakeFiles/LRegression.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable LRegression"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LRegression.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LRegression.dir/build: LRegression
.PHONY : CMakeFiles/LRegression.dir/build

CMakeFiles/LRegression.dir/requires: CMakeFiles/LRegression.dir/src/main.cpp.o.requires
CMakeFiles/LRegression.dir/requires: CMakeFiles/LRegression.dir/src/LRegression.cpp.o.requires
.PHONY : CMakeFiles/LRegression.dir/requires

CMakeFiles/LRegression.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LRegression.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LRegression.dir/clean

CMakeFiles/LRegression.dir/depend:
	cd /home/idriver/study/LRegression && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/idriver/study/LRegression /home/idriver/study/LRegression /home/idriver/study/LRegression /home/idriver/study/LRegression /home/idriver/study/LRegression/CMakeFiles/LRegression.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LRegression.dir/depend

