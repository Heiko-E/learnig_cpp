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

# target_include_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# defines targets and sources
add_subdirectory(src)
# enable testing and define tests
enable_testing()
add_subdirectory(test)
