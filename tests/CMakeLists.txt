enable_testing()

# find packages

list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND REQUIRED_LIBRARIES pcalib)

find_package(GTest REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${GTEST_LIBRARIES})
list(APPEND REQUIRED_LIBRARIES pthread)

# define c++ sources

set(CPP_SOURCES
  camera_test.cpp
  calibration_test.cpp
  calibration_reader_test.cpp
  calibration_writer_test.cpp
  controller_test.cpp
  exception_test.cpp
  linear_response_test.cpp
  poly_response_test.cpp
  response_test.cpp
)

# build library

add_executable(tests tests.cpp ${CPP_SOURCES})
target_link_libraries(tests ${REQUIRED_LIBRARIES})
target_include_directories(tests PUBLIC ${REQUIRED_INCLUDE_DIRS})