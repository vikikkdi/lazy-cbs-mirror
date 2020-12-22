cmake_minimum_required(VERSION 3.10)
project(lazycbs)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED 1)

# Set C++ version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

add_subdirectory(library/geas)
add_subdirectory(library/yaml-cpp)

add_library(lazycbs INTERFACE)
target_include_directories(
  lazycbs
  INTERFACE 
  include 
  library/geas/include
  library/yaml-cpp/include
)

file(GLOB LAZY_CBS_SOURCES "include/lazycbs/*")
file(GLOB GEAS_SOURCES "library/geas/*")

add_executable(
  lazycbs_mapf
  example/lazycbs_test.cpp
  ${LAZY_CBS_SOURCES} ${GEAS_SOURCES}
)
target_link_libraries(
  lazycbs_mapf
  lazycbs
  geas
  yaml-cpp
  ${Boost_LIBRARIES}
)
