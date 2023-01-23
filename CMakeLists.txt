cmake_minimum_required(VERSION 3.16)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

project(LearnigCpp
  VERSION
    1.0
  DESCRIPTION
    "C++ playground"
  LANGUAGES
    CXX
)
add_executable(sum src/sum.cpp)
add_executable(graph src/graph.cpp)
