# python/CMakeLists.txt
cmake_minimum_required(VERSION 3.25)

project(roboflex_realsense_ext VERSION 0.1.1 DESCRIPTION "roboflex realsense python wrapping")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# If you don't do this, then the pybind11_add_module will think it's
# standalone and will not link correctly.
set(PYBIND11_CPP_STANDARD -std=c++20)


# -------------------- 
# Resolve dependencies

find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)

include(FetchContent)

# download and build pybind11
FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# download and build xtensor_python
FetchContent_Declare(xtensor-python
    GIT_REPOSITORY https://github.com/xtensor-stack/xtensor-python.git
    GIT_TAG        0.26.1
)
FetchContent_MakeAvailable(xtensor-python)


# -------------------- 
# Define the library

# Use the pybind11 provided function to create a module.
pybind11_add_module(roboflex_realsense_ext
    pybindings.cpp
)

# Link against your the library and any necessary dependencies
target_link_libraries(roboflex_realsense_ext PUBLIC 
    roboflex_realsense
    xtensor-python
    realsense2 # we seem to need this to get librealsense2.so deployed
)

# If you have specific compile definitions or options for just the Python module
# target_compile_definitions(roboflex_realsense_ext PRIVATE SOME_DEFINITION)

# You can set properties for the target if necessary.
set_target_properties(roboflex_realsense_ext PROPERTIES
    POSITION_INDEPENDENT_CODE ON

    # We want to use an explicit RPATH, so we can
    # find librealsense2.so where we installed it.
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
)

# Add a custom command to copy librealsense2.so after building
add_custom_command(TARGET roboflex_realsense_ext POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:realsense2> 
        $<TARGET_FILE_DIR:roboflex_realsense_ext>/librealsense2.so.2.54.2
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE_DIR:roboflex_realsense_ext>/librealsense2.so.2.54.2
        $<TARGET_FILE_DIR:roboflex_realsense_ext>/librealsense2.so.2.54
)

# -------------------- 
# install

# Install the generated Python module to the desired destination.
# This installs the compiled module.
# install(TARGETS roboflex_realsense_ext
#     LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/roboflex/realsense
# )

# # Install the auxiliary Python files
# install(FILES
#     __init__.py
#     DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/roboflex/realsense
# )
