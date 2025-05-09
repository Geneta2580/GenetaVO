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
CMAKE_SOURCE_DIR = /home/geneta/Project/geneta_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/geneta/Project/geneta_slam/build

# Include any dependencies generated for this target.
include src/CMakeFiles/myslam.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/myslam.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/myslam.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/myslam.dir/flags.make

src/CMakeFiles/myslam.dir/dataset.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/dataset.cpp.o: /home/geneta/Project/geneta_slam/src/dataset.cpp
src/CMakeFiles/myslam.dir/dataset.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/myslam.dir/dataset.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/dataset.cpp.o -MF CMakeFiles/myslam.dir/dataset.cpp.o.d -o CMakeFiles/myslam.dir/dataset.cpp.o -c /home/geneta/Project/geneta_slam/src/dataset.cpp

src/CMakeFiles/myslam.dir/dataset.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/dataset.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/dataset.cpp > CMakeFiles/myslam.dir/dataset.cpp.i

src/CMakeFiles/myslam.dir/dataset.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/dataset.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/dataset.cpp -o CMakeFiles/myslam.dir/dataset.cpp.s

src/CMakeFiles/myslam.dir/frame.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/frame.cpp.o: /home/geneta/Project/geneta_slam/src/frame.cpp
src/CMakeFiles/myslam.dir/frame.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/myslam.dir/frame.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/frame.cpp.o -MF CMakeFiles/myslam.dir/frame.cpp.o.d -o CMakeFiles/myslam.dir/frame.cpp.o -c /home/geneta/Project/geneta_slam/src/frame.cpp

src/CMakeFiles/myslam.dir/frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/frame.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/frame.cpp > CMakeFiles/myslam.dir/frame.cpp.i

src/CMakeFiles/myslam.dir/frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/frame.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/frame.cpp -o CMakeFiles/myslam.dir/frame.cpp.s

src/CMakeFiles/myslam.dir/feature.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/feature.cpp.o: /home/geneta/Project/geneta_slam/src/feature.cpp
src/CMakeFiles/myslam.dir/feature.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/myslam.dir/feature.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/feature.cpp.o -MF CMakeFiles/myslam.dir/feature.cpp.o.d -o CMakeFiles/myslam.dir/feature.cpp.o -c /home/geneta/Project/geneta_slam/src/feature.cpp

src/CMakeFiles/myslam.dir/feature.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/feature.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/feature.cpp > CMakeFiles/myslam.dir/feature.cpp.i

src/CMakeFiles/myslam.dir/feature.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/feature.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/feature.cpp -o CMakeFiles/myslam.dir/feature.cpp.s

src/CMakeFiles/myslam.dir/mappoint.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/mappoint.cpp.o: /home/geneta/Project/geneta_slam/src/mappoint.cpp
src/CMakeFiles/myslam.dir/mappoint.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/myslam.dir/mappoint.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/mappoint.cpp.o -MF CMakeFiles/myslam.dir/mappoint.cpp.o.d -o CMakeFiles/myslam.dir/mappoint.cpp.o -c /home/geneta/Project/geneta_slam/src/mappoint.cpp

src/CMakeFiles/myslam.dir/mappoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/mappoint.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/mappoint.cpp > CMakeFiles/myslam.dir/mappoint.cpp.i

src/CMakeFiles/myslam.dir/mappoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/mappoint.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/mappoint.cpp -o CMakeFiles/myslam.dir/mappoint.cpp.s

src/CMakeFiles/myslam.dir/config.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/config.cpp.o: /home/geneta/Project/geneta_slam/src/config.cpp
src/CMakeFiles/myslam.dir/config.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/myslam.dir/config.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/config.cpp.o -MF CMakeFiles/myslam.dir/config.cpp.o.d -o CMakeFiles/myslam.dir/config.cpp.o -c /home/geneta/Project/geneta_slam/src/config.cpp

src/CMakeFiles/myslam.dir/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/config.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/config.cpp > CMakeFiles/myslam.dir/config.cpp.i

src/CMakeFiles/myslam.dir/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/config.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/config.cpp -o CMakeFiles/myslam.dir/config.cpp.s

src/CMakeFiles/myslam.dir/frontend.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/frontend.cpp.o: /home/geneta/Project/geneta_slam/src/frontend.cpp
src/CMakeFiles/myslam.dir/frontend.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/myslam.dir/frontend.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/frontend.cpp.o -MF CMakeFiles/myslam.dir/frontend.cpp.o.d -o CMakeFiles/myslam.dir/frontend.cpp.o -c /home/geneta/Project/geneta_slam/src/frontend.cpp

src/CMakeFiles/myslam.dir/frontend.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/frontend.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/frontend.cpp > CMakeFiles/myslam.dir/frontend.cpp.i

src/CMakeFiles/myslam.dir/frontend.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/frontend.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/frontend.cpp -o CMakeFiles/myslam.dir/frontend.cpp.s

src/CMakeFiles/myslam.dir/camera.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/camera.cpp.o: /home/geneta/Project/geneta_slam/src/camera.cpp
src/CMakeFiles/myslam.dir/camera.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/myslam.dir/camera.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/camera.cpp.o -MF CMakeFiles/myslam.dir/camera.cpp.o.d -o CMakeFiles/myslam.dir/camera.cpp.o -c /home/geneta/Project/geneta_slam/src/camera.cpp

src/CMakeFiles/myslam.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/camera.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/camera.cpp > CMakeFiles/myslam.dir/camera.cpp.i

src/CMakeFiles/myslam.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/camera.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/camera.cpp -o CMakeFiles/myslam.dir/camera.cpp.s

src/CMakeFiles/myslam.dir/map.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/map.cpp.o: /home/geneta/Project/geneta_slam/src/map.cpp
src/CMakeFiles/myslam.dir/map.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/CMakeFiles/myslam.dir/map.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/map.cpp.o -MF CMakeFiles/myslam.dir/map.cpp.o.d -o CMakeFiles/myslam.dir/map.cpp.o -c /home/geneta/Project/geneta_slam/src/map.cpp

src/CMakeFiles/myslam.dir/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/map.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/map.cpp > CMakeFiles/myslam.dir/map.cpp.i

src/CMakeFiles/myslam.dir/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/map.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/map.cpp -o CMakeFiles/myslam.dir/map.cpp.s

src/CMakeFiles/myslam.dir/viewer.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/viewer.cpp.o: /home/geneta/Project/geneta_slam/src/viewer.cpp
src/CMakeFiles/myslam.dir/viewer.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/CMakeFiles/myslam.dir/viewer.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/viewer.cpp.o -MF CMakeFiles/myslam.dir/viewer.cpp.o.d -o CMakeFiles/myslam.dir/viewer.cpp.o -c /home/geneta/Project/geneta_slam/src/viewer.cpp

src/CMakeFiles/myslam.dir/viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/viewer.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/viewer.cpp > CMakeFiles/myslam.dir/viewer.cpp.i

src/CMakeFiles/myslam.dir/viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/viewer.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/viewer.cpp -o CMakeFiles/myslam.dir/viewer.cpp.s

src/CMakeFiles/myslam.dir/backend.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/backend.cpp.o: /home/geneta/Project/geneta_slam/src/backend.cpp
src/CMakeFiles/myslam.dir/backend.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/CMakeFiles/myslam.dir/backend.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/backend.cpp.o -MF CMakeFiles/myslam.dir/backend.cpp.o.d -o CMakeFiles/myslam.dir/backend.cpp.o -c /home/geneta/Project/geneta_slam/src/backend.cpp

src/CMakeFiles/myslam.dir/backend.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/backend.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/backend.cpp > CMakeFiles/myslam.dir/backend.cpp.i

src/CMakeFiles/myslam.dir/backend.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/backend.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/backend.cpp -o CMakeFiles/myslam.dir/backend.cpp.s

src/CMakeFiles/myslam.dir/loopclosing.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/loopclosing.cpp.o: /home/geneta/Project/geneta_slam/src/loopclosing.cpp
src/CMakeFiles/myslam.dir/loopclosing.cpp.o: src/CMakeFiles/myslam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/CMakeFiles/myslam.dir/loopclosing.cpp.o"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/myslam.dir/loopclosing.cpp.o -MF CMakeFiles/myslam.dir/loopclosing.cpp.o.d -o CMakeFiles/myslam.dir/loopclosing.cpp.o -c /home/geneta/Project/geneta_slam/src/loopclosing.cpp

src/CMakeFiles/myslam.dir/loopclosing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/myslam.dir/loopclosing.cpp.i"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geneta/Project/geneta_slam/src/loopclosing.cpp > CMakeFiles/myslam.dir/loopclosing.cpp.i

src/CMakeFiles/myslam.dir/loopclosing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/loopclosing.cpp.s"
	cd /home/geneta/Project/geneta_slam/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geneta/Project/geneta_slam/src/loopclosing.cpp -o CMakeFiles/myslam.dir/loopclosing.cpp.s

# Object files for target myslam
myslam_OBJECTS = \
"CMakeFiles/myslam.dir/dataset.cpp.o" \
"CMakeFiles/myslam.dir/frame.cpp.o" \
"CMakeFiles/myslam.dir/feature.cpp.o" \
"CMakeFiles/myslam.dir/mappoint.cpp.o" \
"CMakeFiles/myslam.dir/config.cpp.o" \
"CMakeFiles/myslam.dir/frontend.cpp.o" \
"CMakeFiles/myslam.dir/camera.cpp.o" \
"CMakeFiles/myslam.dir/map.cpp.o" \
"CMakeFiles/myslam.dir/viewer.cpp.o" \
"CMakeFiles/myslam.dir/backend.cpp.o" \
"CMakeFiles/myslam.dir/loopclosing.cpp.o"

# External object files for target myslam
myslam_EXTERNAL_OBJECTS =

/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/dataset.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/frame.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/feature.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/mappoint.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/config.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/frontend.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/camera.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/map.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/viewer.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/backend.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/loopclosing.cpp.o
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/build.make
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_highgui.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_ml.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_objdetect.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_photo.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_stitching.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_video.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_videoio.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_glgeometry.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_plot.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_python.so
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_scene.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_tools.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_video.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: Thirdparty/DBow3/src/libDBoW3.so.0.0.1
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_highgui.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_ml.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_objdetect.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_photo.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_stitching.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_video.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_calib3d.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_dnn.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_features2d.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_flann.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_videoio.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_imgproc.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_core.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_calib3d.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_dnn.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_features2d.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_flann.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_imgproc.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libopencv_core.so.4.10.0
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_geometry.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libtinyobj.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_display.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_vars.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_windowing.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_opengl.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libEGL.so
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libOpenGL.so
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libepoxy.so
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_image.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_packetstream.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: /usr/local/lib/libpango_core.so.0.9.3
/home/geneta/Project/geneta_slam/lib/libmyslam.so: src/CMakeFiles/myslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/geneta/Project/geneta_slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX shared library /home/geneta/Project/geneta_slam/lib/libmyslam.so"
	cd /home/geneta/Project/geneta_slam/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/myslam.dir/build: /home/geneta/Project/geneta_slam/lib/libmyslam.so
.PHONY : src/CMakeFiles/myslam.dir/build

src/CMakeFiles/myslam.dir/clean:
	cd /home/geneta/Project/geneta_slam/build/src && $(CMAKE_COMMAND) -P CMakeFiles/myslam.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/myslam.dir/clean

src/CMakeFiles/myslam.dir/depend:
	cd /home/geneta/Project/geneta_slam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/geneta/Project/geneta_slam /home/geneta/Project/geneta_slam/src /home/geneta/Project/geneta_slam/build /home/geneta/Project/geneta_slam/build/src /home/geneta/Project/geneta_slam/build/src/CMakeFiles/myslam.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/CMakeFiles/myslam.dir/depend

