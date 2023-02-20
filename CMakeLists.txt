cmake_minimum_required(VERSION 3.16)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

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

add_executable(test_sum test/test_sum.cc)

enable_testing()

target_link_libraries(
  test_sum
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_sum)
