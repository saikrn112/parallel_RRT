cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(cmake_and_cuda LANGUAGES CXX CUDA)



# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME rrtstar-standalone)
include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)

#tell cmake to build these subdirectories
add_subdirectory(src)
