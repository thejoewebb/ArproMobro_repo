# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/Debug

# Include any dependencies generated for this target.
include CMakeFiles/move_robot.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/move_robot.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/move_robot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/move_robot.dir/flags.make

CMakeFiles/move_robot.dir/src/main.cpp.o: CMakeFiles/move_robot.dir/flags.make
CMakeFiles/move_robot.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/move_robot.dir/src/main.cpp.o: CMakeFiles/move_robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/move_robot.dir/src/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/move_robot.dir/src/main.cpp.o -MF CMakeFiles/move_robot.dir/src/main.cpp.o.d -o CMakeFiles/move_robot.dir/src/main.cpp.o -c /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/src/main.cpp

CMakeFiles/move_robot.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/move_robot.dir/src/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/src/main.cpp > CMakeFiles/move_robot.dir/src/main.cpp.i

CMakeFiles/move_robot.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/move_robot.dir/src/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/src/main.cpp -o CMakeFiles/move_robot.dir/src/main.cpp.s

# Object files for target move_robot
move_robot_OBJECTS = \
"CMakeFiles/move_robot.dir/src/main.cpp.o"

# External object files for target move_robot
move_robot_EXTERNAL_OBJECTS =

move_robot: CMakeFiles/move_robot.dir/src/main.cpp.o
move_robot: CMakeFiles/move_robot.dir/build.make
move_robot: liblab2.so
move_robot: CMakeFiles/move_robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable move_robot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/move_robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/move_robot.dir/build: move_robot
.PHONY : CMakeFiles/move_robot.dir/build

CMakeFiles/move_robot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/move_robot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/move_robot.dir/clean

CMakeFiles/move_robot.dir/depend:
	cd /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/Debug /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/Debug /home/thejoewebb/Documents/ECN/ARPRO_git/ArproMobro_repo/Debug/CMakeFiles/move_robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/move_robot.dir/depend

