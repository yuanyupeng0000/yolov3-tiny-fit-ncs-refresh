# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build

# Include any dependencies generated for this target.
include CMakeFiles/object_detection_yolov3_async.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/object_detection_yolov3_async.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/object_detection_yolov3_async.dir/flags.make

CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o: CMakeFiles/object_detection_yolov3_async.dir/flags.make
CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o: ../intel_dldt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o -c /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/intel_dldt.cpp

CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/intel_dldt.cpp > CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.i

CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/intel_dldt.cpp -o CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.s

CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.requires:

.PHONY : CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.requires

CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.provides: CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.requires
	$(MAKE) -f CMakeFiles/object_detection_yolov3_async.dir/build.make CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.provides.build
.PHONY : CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.provides

CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.provides.build: CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o


CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o: CMakeFiles/object_detection_yolov3_async.dir/flags.make
CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o: ../Common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o -c /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/Common.cpp

CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/Common.cpp > CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.i

CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/Common.cpp -o CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.s

CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.requires:

.PHONY : CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.requires

CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.provides: CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.requires
	$(MAKE) -f CMakeFiles/object_detection_yolov3_async.dir/build.make CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.provides.build
.PHONY : CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.provides

CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.provides.build: CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o


CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o: CMakeFiles/object_detection_yolov3_async.dir/flags.make
CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o -c /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/main.cpp

CMakeFiles/object_detection_yolov3_async.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_detection_yolov3_async.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/main.cpp > CMakeFiles/object_detection_yolov3_async.dir/main.cpp.i

CMakeFiles/object_detection_yolov3_async.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_detection_yolov3_async.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/main.cpp -o CMakeFiles/object_detection_yolov3_async.dir/main.cpp.s

CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.requires

CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.provides: CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/object_detection_yolov3_async.dir/build.make CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.provides

CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.provides.build: CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o


CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o: CMakeFiles/object_detection_yolov3_async.dir/flags.make
CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o: ../detector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o -c /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/detector.cpp

CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/detector.cpp > CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.i

CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/detector.cpp -o CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.s

CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.requires:

.PHONY : CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.requires

CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.provides: CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.requires
	$(MAKE) -f CMakeFiles/object_detection_yolov3_async.dir/build.make CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.provides.build
.PHONY : CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.provides

CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.provides.build: CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o


# Object files for target object_detection_yolov3_async
object_detection_yolov3_async_OBJECTS = \
"CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o" \
"CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o" \
"CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o" \
"CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o"

# External object files for target object_detection_yolov3_async
object_detection_yolov3_async_EXTERNAL_OBJECTS =

libobject_detection_yolov3_async.so: CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o
libobject_detection_yolov3_async.so: CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o
libobject_detection_yolov3_async.so: CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o
libobject_detection_yolov3_async.so: CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o
libobject_detection_yolov3_async.so: CMakeFiles/object_detection_yolov3_async.dir/build.make
libobject_detection_yolov3_async.so: CMakeFiles/object_detection_yolov3_async.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library libobject_detection_yolov3_async.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/object_detection_yolov3_async.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/object_detection_yolov3_async.dir/build: libobject_detection_yolov3_async.so

.PHONY : CMakeFiles/object_detection_yolov3_async.dir/build

CMakeFiles/object_detection_yolov3_async.dir/requires: CMakeFiles/object_detection_yolov3_async.dir/intel_dldt.cpp.o.requires
CMakeFiles/object_detection_yolov3_async.dir/requires: CMakeFiles/object_detection_yolov3_async.dir/Common.cpp.o.requires
CMakeFiles/object_detection_yolov3_async.dir/requires: CMakeFiles/object_detection_yolov3_async.dir/main.cpp.o.requires
CMakeFiles/object_detection_yolov3_async.dir/requires: CMakeFiles/object_detection_yolov3_async.dir/detector.cpp.o.requires

.PHONY : CMakeFiles/object_detection_yolov3_async.dir/requires

CMakeFiles/object_detection_yolov3_async.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/object_detection_yolov3_async.dir/cmake_clean.cmake
.PHONY : CMakeFiles/object_detection_yolov3_async.dir/clean

CMakeFiles/object_detection_yolov3_async.dir/depend:
	cd /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build /data/github_repos/yolov3-tiny-fit-ncs/ncs2/OpenVINO/inference_engine/samples/object_detection_demo_yolov3_async/build/CMakeFiles/object_detection_yolov3_async.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/object_detection_yolov3_async.dir/depend

