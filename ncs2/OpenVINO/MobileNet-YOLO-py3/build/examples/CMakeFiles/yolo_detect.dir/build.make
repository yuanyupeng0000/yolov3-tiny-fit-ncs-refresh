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
CMAKE_SOURCE_DIR = /data/github_repos/MobileNet-YOLO-py3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /data/github_repos/MobileNet-YOLO-py3/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/yolo_detect.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/yolo_detect.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/yolo_detect.dir/flags.make

examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o: examples/CMakeFiles/yolo_detect.dir/flags.make
examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o: ../examples/yolo/yolo_detect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/github_repos/MobileNet-YOLO-py3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o"
	cd /data/github_repos/MobileNet-YOLO-py3/build/examples && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o -c /data/github_repos/MobileNet-YOLO-py3/examples/yolo/yolo_detect.cpp

examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.i"
	cd /data/github_repos/MobileNet-YOLO-py3/build/examples && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/github_repos/MobileNet-YOLO-py3/examples/yolo/yolo_detect.cpp > CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.i

examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.s"
	cd /data/github_repos/MobileNet-YOLO-py3/build/examples && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/github_repos/MobileNet-YOLO-py3/examples/yolo/yolo_detect.cpp -o CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.s

examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.requires:

.PHONY : examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.requires

examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.provides: examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/yolo_detect.dir/build.make examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.provides.build
.PHONY : examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.provides

examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.provides.build: examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o


# Object files for target yolo_detect
yolo_detect_OBJECTS = \
"CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o"

# External object files for target yolo_detect
yolo_detect_EXTERNAL_OBJECTS =

examples/yolo/yolo_detect: examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o
examples/yolo/yolo_detect: examples/CMakeFiles/yolo_detect.dir/build.make
examples/yolo/yolo_detect: lib/libcaffe.so.1.0.0
examples/yolo/yolo_detect: lib/libcaffeproto.a
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libboost_system.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libboost_thread.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libpthread.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libglog.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libgflags.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_hl.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libsz.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libz.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libdl.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libm.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libpthread.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libglog.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libgflags.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_hl.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libsz.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libz.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libdl.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libm.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libprotobuf.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/liblmdb.so
examples/yolo/yolo_detect: /usr/local/cuda-8.0/lib64/libcudart.so
examples/yolo/yolo_detect: /usr/local/cuda-8.0/lib64/libcurand.so
examples/yolo/yolo_detect: /usr/local/cuda-8.0/lib64/libcublas.so
examples/yolo/yolo_detect: /usr/local/lib/libopencv_highgui.so.3.4.0
examples/yolo/yolo_detect: /usr/local/lib/libopencv_videoio.so.3.4.0
examples/yolo/yolo_detect: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
examples/yolo/yolo_detect: /usr/local/lib/libopencv_imgproc.so.3.4.0
examples/yolo/yolo_detect: /usr/local/lib/libopencv_core.so.3.4.0
examples/yolo/yolo_detect: /usr/local/lib/libopencv_cudev.so.3.4.0
examples/yolo/yolo_detect: /usr/lib/liblapack.so
examples/yolo/yolo_detect: /usr/lib/libcblas.so
examples/yolo/yolo_detect: /usr/lib/libatlas.so
examples/yolo/yolo_detect: /usr/lib/x86_64-linux-gnu/libboost_python-py35.so
examples/yolo/yolo_detect: examples/CMakeFiles/yolo_detect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/data/github_repos/MobileNet-YOLO-py3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable yolo/yolo_detect"
	cd /data/github_repos/MobileNet-YOLO-py3/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yolo_detect.dir/link.txt --verbose=$(VERBOSE)
	cd /data/github_repos/MobileNet-YOLO-py3/build/examples && ln -sf /data/github_repos/MobileNet-YOLO-py3/build/examples/yolo/yolo_detect /data/github_repos/MobileNet-YOLO-py3/build/examples/yolo/yolo_detect.bin

# Rule to build all files generated by this target.
examples/CMakeFiles/yolo_detect.dir/build: examples/yolo/yolo_detect

.PHONY : examples/CMakeFiles/yolo_detect.dir/build

examples/CMakeFiles/yolo_detect.dir/requires: examples/CMakeFiles/yolo_detect.dir/yolo/yolo_detect.cpp.o.requires

.PHONY : examples/CMakeFiles/yolo_detect.dir/requires

examples/CMakeFiles/yolo_detect.dir/clean:
	cd /data/github_repos/MobileNet-YOLO-py3/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/yolo_detect.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/yolo_detect.dir/clean

examples/CMakeFiles/yolo_detect.dir/depend:
	cd /data/github_repos/MobileNet-YOLO-py3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /data/github_repos/MobileNet-YOLO-py3 /data/github_repos/MobileNet-YOLO-py3/examples /data/github_repos/MobileNet-YOLO-py3/build /data/github_repos/MobileNet-YOLO-py3/build/examples /data/github_repos/MobileNet-YOLO-py3/build/examples/CMakeFiles/yolo_detect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/yolo_detect.dir/depend

