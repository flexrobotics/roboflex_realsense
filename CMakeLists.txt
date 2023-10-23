cmake_minimum_required(VERSION 3.25)

project(roboflex_realsense VERSION 0.1.1 DESCRIPTION "roboflex realsense2 sensor")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# -------------------- 
# Resolve dependencies

# roboflex core
include(FetchContent)


# Realsense2: we'd like to use whatever
# the user has installed, but fetch and 
# build it if they don't have it.

# ... it seems to cause problems ...
find_package(realsense2 QUIET)
if(NOT realsense2_FOUND)

    message(">>>>>>>>>> REALSENSE NOT FOUND, CMAKE_CURRENT_BINARY_DIR is ${CMAKE_CURRENT_BINARY_DIR} <<<<<<<<<<<<<<<")

    # realsense doesn't compile with c++20 urg 
    set(CMAKE_CXX_STANDARD 11)

    # Set desired cache variables for librealsense build
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/install" CACHE PATH "Installation directory for librealsense" FORCE)

    FetchContent_Declare(
        realsense2
        GIT_REPOSITORY https://github.com/IntelRealSense/librealsense.git
        GIT_TAG        v2.54.2
        #GIT_TAG        master
    )
    FetchContent_MakeAvailable(realsense2)

    set(CMAKE_CXX_STANDARD 20)
else()
    message(">>>>>>>>>>>> REALSENSE FOUND THROUGH find_package(realsense2 QUIET) <<<<<<<<<<<<<<<<")
endif()


# download and build roboflex_core
FetchContent_Declare(roboflex_core
    GIT_REPOSITORY https://github.com/flexrobotics/roboflex.git
    GIT_TAG        main
)
set(BUILD_ROBOFLEX_PYTHON_EXT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(roboflex_core)


# -------------------- 
# Define the library

add_library(roboflex_realsense STATIC
    src/realsense.cpp
    include/roboflex_realsense/realsense.h
)

# Set some properties on our library
set_property(TARGET roboflex_realsense PROPERTY 
    POSITION_INDEPENDENT_CODE ON
)

# Include directories when we compile our library
target_include_directories(roboflex_realsense PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
)

# Link against the necessary libraries
target_link_libraries(roboflex_realsense PUBLIC 
    roboflex_core 
    realsense2

    # Unfortunately, we need to link against all these libraries
    # because they are used by roboflex_core, and cmake doesn't
    # seem to be transitivily doing it's thing. Urgh, someone help.
    flatbuffers_util 
    xtensor 
    xsimd 
    xtl
    Eigen3::Eigen
)


# -------------------- 
# Examples

# run_realsense_cpp example
add_executable(run_realsense_cpp examples/run_realsense_cpp.cpp)
target_link_libraries(run_realsense_cpp PRIVATE 
    roboflex_core 
    roboflex_realsense
)


# -------------------- 
# install

# If you need to install the realsense library
install(TARGETS roboflex_realsense 
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(DIRECTORY include/roboflex_realsense
    DESTINATION include
)


# --------------------
# build python bindings

add_subdirectory(python)
