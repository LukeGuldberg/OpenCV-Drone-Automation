# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.25

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = C:\msys64\mingw64\bin\cmake.exe

# The command to remove a file.
RM = C:\msys64\mingw64\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\lukeg\code\opencv-msys

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\lukeg\code\opencv-msys\build

# Include any dependencies generated for this target.
include CMakeFiles/webcamset.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/webcamset.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/webcamset.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/webcamset.dir/flags.make

CMakeFiles/webcamset.dir/webcamset.cpp.obj: CMakeFiles/webcamset.dir/flags.make
CMakeFiles/webcamset.dir/webcamset.cpp.obj: CMakeFiles/webcamset.dir/includes_CXX.rsp
CMakeFiles/webcamset.dir/webcamset.cpp.obj: C:/Users/lukeg/code/opencv-msys/webcamset.cpp
CMakeFiles/webcamset.dir/webcamset.cpp.obj: CMakeFiles/webcamset.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\lukeg\code\opencv-msys\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/webcamset.dir/webcamset.cpp.obj"
	C:\msys64\mingw64\bin\x86_64-w64-mingw32-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/webcamset.dir/webcamset.cpp.obj -MF CMakeFiles\webcamset.dir\webcamset.cpp.obj.d -o CMakeFiles\webcamset.dir\webcamset.cpp.obj -c C:\Users\lukeg\code\opencv-msys\webcamset.cpp

CMakeFiles/webcamset.dir/webcamset.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/webcamset.dir/webcamset.cpp.i"
	C:\msys64\mingw64\bin\x86_64-w64-mingw32-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\lukeg\code\opencv-msys\webcamset.cpp > CMakeFiles\webcamset.dir\webcamset.cpp.i

CMakeFiles/webcamset.dir/webcamset.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/webcamset.dir/webcamset.cpp.s"
	C:\msys64\mingw64\bin\x86_64-w64-mingw32-g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\lukeg\code\opencv-msys\webcamset.cpp -o CMakeFiles\webcamset.dir\webcamset.cpp.s

# Object files for target webcamset
webcamset_OBJECTS = \
"CMakeFiles/webcamset.dir/webcamset.cpp.obj"

# External object files for target webcamset
webcamset_EXTERNAL_OBJECTS =

webcamset.exe: CMakeFiles/webcamset.dir/webcamset.cpp.obj
webcamset.exe: CMakeFiles/webcamset.dir/build.make
webcamset.exe: C:/msys64/mingw64/lib/libopencv_gapi.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_stitching.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_alphamat.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_aruco.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_barcode.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_bgsegm.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_ccalib.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_cvv.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_dnn_objdetect.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_dnn_superres.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_dpm.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_face.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_freetype.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_fuzzy.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_hdf.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_hfs.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_img_hash.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_intensity_transform.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_line_descriptor.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_mcc.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_ovis.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_quality.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_rapid.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_reg.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_rgbd.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_saliency.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_sfm.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_stereo.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_structured_light.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_superres.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_surface_matching.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_tracking.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_videostab.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_viz.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_wechat_qrcode.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_xfeatures2d.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_xobjdetect.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_xphoto.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_shape.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_highgui.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_datasets.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_plot.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_text.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_ml.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_phase_unwrapping.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_optflow.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_ximgproc.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_video.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_videoio.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_imgcodecs.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_objdetect.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_calib3d.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_dnn.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_features2d.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_flann.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_photo.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_imgproc.dll.a
webcamset.exe: C:/msys64/mingw64/lib/libopencv_core.dll.a
webcamset.exe: CMakeFiles/webcamset.dir/linkLibs.rsp
webcamset.exe: CMakeFiles/webcamset.dir/objects1
webcamset.exe: CMakeFiles/webcamset.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\lukeg\code\opencv-msys\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable webcamset.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\webcamset.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/webcamset.dir/build: webcamset.exe
.PHONY : CMakeFiles/webcamset.dir/build

CMakeFiles/webcamset.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\webcamset.dir\cmake_clean.cmake
.PHONY : CMakeFiles/webcamset.dir/clean

CMakeFiles/webcamset.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\lukeg\code\opencv-msys C:\Users\lukeg\code\opencv-msys C:\Users\lukeg\code\opencv-msys\build C:\Users\lukeg\code\opencv-msys\build C:\Users\lukeg\code\opencv-msys\build\CMakeFiles\webcamset.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/webcamset.dir/depend

