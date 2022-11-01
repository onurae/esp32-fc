# Minimum CMake version
cmake_minimum_required(VERSION 3.5)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include cmake path
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Name of the project
project(esp32-fc)