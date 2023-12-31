# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/build

# Include any dependencies generated for this target.
include CMakeFiles/openMVG_easyexif.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/openMVG_easyexif.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/openMVG_easyexif.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openMVG_easyexif.dir/flags.make

CMakeFiles/openMVG_easyexif.dir/exif.o: CMakeFiles/openMVG_easyexif.dir/flags.make
CMakeFiles/openMVG_easyexif.dir/exif.o: ../exif.cpp
CMakeFiles/openMVG_easyexif.dir/exif.o: CMakeFiles/openMVG_easyexif.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/openMVG_easyexif.dir/exif.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/openMVG_easyexif.dir/exif.o -MF CMakeFiles/openMVG_easyexif.dir/exif.o.d -o CMakeFiles/openMVG_easyexif.dir/exif.o -c /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/exif.cpp

CMakeFiles/openMVG_easyexif.dir/exif.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openMVG_easyexif.dir/exif.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/exif.cpp > CMakeFiles/openMVG_easyexif.dir/exif.i

CMakeFiles/openMVG_easyexif.dir/exif.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openMVG_easyexif.dir/exif.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/exif.cpp -o CMakeFiles/openMVG_easyexif.dir/exif.s

# Object files for target openMVG_easyexif
openMVG_easyexif_OBJECTS = \
"CMakeFiles/openMVG_easyexif.dir/exif.o"

# External object files for target openMVG_easyexif
openMVG_easyexif_EXTERNAL_OBJECTS =

libopenMVG_easyexif.a: CMakeFiles/openMVG_easyexif.dir/exif.o
libopenMVG_easyexif.a: CMakeFiles/openMVG_easyexif.dir/build.make
libopenMVG_easyexif.a: CMakeFiles/openMVG_easyexif.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libopenMVG_easyexif.a"
	$(CMAKE_COMMAND) -P CMakeFiles/openMVG_easyexif.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openMVG_easyexif.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openMVG_easyexif.dir/build: libopenMVG_easyexif.a
.PHONY : CMakeFiles/openMVG_easyexif.dir/build

CMakeFiles/openMVG_easyexif.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openMVG_easyexif.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openMVG_easyexif.dir/clean

CMakeFiles/openMVG_easyexif.dir/depend:
	cd /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/build /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/build /home/xxx/00_project/04_sfm_slam_project/openMVG/src/third_party/easyexif/build/CMakeFiles/openMVG_easyexif.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openMVG_easyexif.dir/depend

