# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.17

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

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2020.3.1\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2020.3.1\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/SpaceGrantOldPi.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SpaceGrantOldPi.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SpaceGrantOldPi.dir/flags.make

CMakeFiles/SpaceGrantOldPi.dir/main.cpp.obj: CMakeFiles/SpaceGrantOldPi.dir/flags.make
CMakeFiles/SpaceGrantOldPi.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SpaceGrantOldPi.dir/main.cpp.obj"
	C:\mingw64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\SpaceGrantOldPi.dir\main.cpp.obj -c C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\main.cpp

CMakeFiles/SpaceGrantOldPi.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SpaceGrantOldPi.dir/main.cpp.i"
	C:\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\main.cpp > CMakeFiles\SpaceGrantOldPi.dir\main.cpp.i

CMakeFiles/SpaceGrantOldPi.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SpaceGrantOldPi.dir/main.cpp.s"
	C:\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\main.cpp -o CMakeFiles\SpaceGrantOldPi.dir\main.cpp.s

CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.obj: CMakeFiles/SpaceGrantOldPi.dir/flags.make
CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.obj: ../robotControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.obj"
	C:\mingw64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\SpaceGrantOldPi.dir\robotControl.cpp.obj -c C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\robotControl.cpp

CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.i"
	C:\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\robotControl.cpp > CMakeFiles\SpaceGrantOldPi.dir\robotControl.cpp.i

CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.s"
	C:\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\robotControl.cpp -o CMakeFiles\SpaceGrantOldPi.dir\robotControl.cpp.s

# Object files for target SpaceGrantOldPi
SpaceGrantOldPi_OBJECTS = \
"CMakeFiles/SpaceGrantOldPi.dir/main.cpp.obj" \
"CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.obj"

# External object files for target SpaceGrantOldPi
SpaceGrantOldPi_EXTERNAL_OBJECTS =

SpaceGrantOldPi.exe: CMakeFiles/SpaceGrantOldPi.dir/main.cpp.obj
SpaceGrantOldPi.exe: CMakeFiles/SpaceGrantOldPi.dir/robotControl.cpp.obj
SpaceGrantOldPi.exe: CMakeFiles/SpaceGrantOldPi.dir/build.make
SpaceGrantOldPi.exe: CMakeFiles/SpaceGrantOldPi.dir/linklibs.rsp
SpaceGrantOldPi.exe: CMakeFiles/SpaceGrantOldPi.dir/objects1.rsp
SpaceGrantOldPi.exe: CMakeFiles/SpaceGrantOldPi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable SpaceGrantOldPi.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\SpaceGrantOldPi.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SpaceGrantOldPi.dir/build: SpaceGrantOldPi.exe

.PHONY : CMakeFiles/SpaceGrantOldPi.dir/build

CMakeFiles/SpaceGrantOldPi.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\SpaceGrantOldPi.dir\cmake_clean.cmake
.PHONY : CMakeFiles/SpaceGrantOldPi.dir/clean

CMakeFiles/SpaceGrantOldPi.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\cmake-build-debug C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\cmake-build-debug C:\Users\colin\OneDrive\Documents\GitHub\SpaceGrantOldPi\cmake-build-debug\CMakeFiles\SpaceGrantOldPi.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SpaceGrantOldPi.dir/depend

