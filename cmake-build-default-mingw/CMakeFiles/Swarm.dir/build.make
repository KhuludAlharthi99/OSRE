# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2021.2.3\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2021.2.3\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\cmake-build-default-mingw"

# Include any dependencies generated for this target.
include CMakeFiles/Swarm.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/Swarm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Swarm.dir/flags.make

CMakeFiles/Swarm.dir/main.cpp.obj: CMakeFiles/Swarm.dir/flags.make
CMakeFiles/Swarm.dir/main.cpp.obj: CMakeFiles/Swarm.dir/includes_CXX.rsp
CMakeFiles/Swarm.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\cmake-build-default-mingw\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Swarm.dir/main.cpp.obj"
	C:\PROGRA~2\MINGW-~1\I686-8~1.0-P\mingw32\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\Swarm.dir\main.cpp.obj -c "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\main.cpp"

CMakeFiles/Swarm.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Swarm.dir/main.cpp.i"
	C:\PROGRA~2\MINGW-~1\I686-8~1.0-P\mingw32\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\main.cpp" > CMakeFiles\Swarm.dir\main.cpp.i

CMakeFiles/Swarm.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Swarm.dir/main.cpp.s"
	C:\PROGRA~2\MINGW-~1\I686-8~1.0-P\mingw32\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\main.cpp" -o CMakeFiles\Swarm.dir\main.cpp.s

# Object files for target Swarm
Swarm_OBJECTS = \
"CMakeFiles/Swarm.dir/main.cpp.obj"

# External object files for target Swarm
Swarm_EXTERNAL_OBJECTS =

Swarm.exe: CMakeFiles/Swarm.dir/main.cpp.obj
Swarm.exe: CMakeFiles/Swarm.dir/build.make
Swarm.exe: CMakeFiles/Swarm.dir/linklibs.rsp
Swarm.exe: CMakeFiles/Swarm.dir/objects1.rsp
Swarm.exe: CMakeFiles/Swarm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\cmake-build-default-mingw\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Swarm.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\Swarm.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Swarm.dir/build: Swarm.exe
.PHONY : CMakeFiles/Swarm.dir/build

CMakeFiles/Swarm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Swarm.dir\cmake_clean.cmake
.PHONY : CMakeFiles/Swarm.dir/clean

CMakeFiles/Swarm.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm" "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm" "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\cmake-build-default-mingw" "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\cmake-build-default-mingw" "C:\Users\yd20883\OneDrive - University of Bristol\Desktop\Swarm2\Swarm\cmake-build-default-mingw\CMakeFiles\Swarm.dir\DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/Swarm.dir/depend

