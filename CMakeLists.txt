cmake_minimum_required(VERSION 2.8)
project(Photocalib)

# find packages

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/include)

find_package(GFlags REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${GFlags_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${GFlags_LIBRARIES})

find_package(GLog REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${GLog_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${GLog_LIBRARIES})

# set compiler flags

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# define c++ configs

set(CPP_CONFIG_DIR include/photocalib)

set(CPP_CONFIGS
  ${CPP_CONFIG_DIR}/photocalib.h
)

# define c++ headers

set(CPP_HEADER_DIR include/photocalib)

set(CPP_HEADERS
  ${CPP_HEADER_DIR}/camera.h
)

# define c++ sources

set(CPP_SOURCE_DIR src)

set(CPP_SOURCES
  ${CPP_SOURCE_DIR}/camera.cpp
)

# build library

add_library(photocalib SHARED ${CPP_SOURCES})
target_link_libraries(photocalib ${REQUIRED_LIBRARIES})
target_include_directories(photocalib PUBLIC ${REQUIRED_INCLUDE_DIRS})

# build applications

option(PHOTOCALIB_BUILD_APPS "Build applications" ON)

if(PHOTOCALIB_BUILD_APPS)
  add_subdirectory(apps)
endif()

# build unit tests

option(PHOTOCALIB_BUILD_TESTS "Build unit tests" OFF)

if(PHOTOCALIB_BUILD_TESTS)
  add_subdirectory(tests)
endif()