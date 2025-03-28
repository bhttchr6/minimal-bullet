# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_SOURCE_DIR = /home/anurag/Documents/Projects/robotics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anurag/Documents/Projects/robotics/build

# Include any dependencies generated for this target.
include CMakeFiles/testBullet1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/testBullet1.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/testBullet1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testBullet1.dir/flags.make

CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o: CMakeFiles/testBullet1.dir/flags.make
CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o: /home/anurag/Documents/Projects/robotics/src/testBullet1.cpp
CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o: CMakeFiles/testBullet1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/anurag/Documents/Projects/robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o -MF CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o.d -o CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o -c /home/anurag/Documents/Projects/robotics/src/testBullet1.cpp

CMakeFiles/testBullet1.dir/src/testBullet1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/testBullet1.dir/src/testBullet1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anurag/Documents/Projects/robotics/src/testBullet1.cpp > CMakeFiles/testBullet1.dir/src/testBullet1.cpp.i

CMakeFiles/testBullet1.dir/src/testBullet1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/testBullet1.dir/src/testBullet1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anurag/Documents/Projects/robotics/src/testBullet1.cpp -o CMakeFiles/testBullet1.dir/src/testBullet1.cpp.s

CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o: CMakeFiles/testBullet1.dir/flags.make
CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o: /home/anurag/Documents/Projects/robotics/third/glad/src/glad.c
CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o: CMakeFiles/testBullet1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/anurag/Documents/Projects/robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o -MF CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o.d -o CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o -c /home/anurag/Documents/Projects/robotics/third/glad/src/glad.c

CMakeFiles/testBullet1.dir/third/glad/src/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/testBullet1.dir/third/glad/src/glad.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/anurag/Documents/Projects/robotics/third/glad/src/glad.c > CMakeFiles/testBullet1.dir/third/glad/src/glad.c.i

CMakeFiles/testBullet1.dir/third/glad/src/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/testBullet1.dir/third/glad/src/glad.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/anurag/Documents/Projects/robotics/third/glad/src/glad.c -o CMakeFiles/testBullet1.dir/third/glad/src/glad.c.s

# Object files for target testBullet1
testBullet1_OBJECTS = \
"CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o" \
"CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o"

# External object files for target testBullet1
testBullet1_EXTERNAL_OBJECTS =

testBullet1: CMakeFiles/testBullet1.dir/src/testBullet1.cpp.o
testBullet1: CMakeFiles/testBullet1.dir/third/glad/src/glad.c.o
testBullet1: CMakeFiles/testBullet1.dir/build.make
testBullet1: /usr/lib/x86_64-linux-gnu/libGL.so
testBullet1: third/glfw/src/libglfw3.a
testBullet1: third/vendored/SDL/libSDL3.a
testBullet1: /home/anurag/Documents/Projects/robotics/third/bullet3/build_cmake/local_install/lib/libBulletDynamics.a
testBullet1: /home/anurag/Documents/Projects/robotics/third/bullet3/build_cmake/local_install/lib/libBulletCollision.a
testBullet1: /home/anurag/Documents/Projects/robotics/third/bullet3/build_cmake/local_install/lib/libLinearMath.a
testBullet1: /usr/lib/x86_64-linux-gnu/librt.a
testBullet1: /usr/lib/x86_64-linux-gnu/libm.so
testBullet1: CMakeFiles/testBullet1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/anurag/Documents/Projects/robotics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable testBullet1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testBullet1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testBullet1.dir/build: testBullet1
.PHONY : CMakeFiles/testBullet1.dir/build

CMakeFiles/testBullet1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testBullet1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testBullet1.dir/clean

CMakeFiles/testBullet1.dir/depend:
	cd /home/anurag/Documents/Projects/robotics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anurag/Documents/Projects/robotics /home/anurag/Documents/Projects/robotics /home/anurag/Documents/Projects/robotics/build /home/anurag/Documents/Projects/robotics/build /home/anurag/Documents/Projects/robotics/build/CMakeFiles/testBullet1.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/testBullet1.dir/depend

