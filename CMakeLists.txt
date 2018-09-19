cmake_minimum_required(VERSION 2.8)
project(pcalib)

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

find_package(Eigen3 REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${EIGEN3_LIBRARIES})

find_package(Ceres REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${CERES_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${CERES_LIBRARIES})

find_package(Calibu REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${Calibu_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${Calibu_LIBRARIES})

find_package(Pangolin REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${Pangolin_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${Pangolin_LIBRARIES})

find_package(TinyXML2 REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${TinyXML2_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${TinyXML2_LIBRARIES})

# set compiler flags

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# define c++ configs

set(CPP_CONFIG_DIR include/pcalib)

set(CPP_CONFIGS
  ${CPP_CONFIG_DIR}/pcalib.h
)

# define c++ headers

set(CPP_HEADER_DIR include/pcalib)

set(CPP_HEADERS
  ${CPP_HEADER_DIR}/calibration.h
  ${CPP_HEADER_DIR}/calibration_reader.h
  ${CPP_HEADER_DIR}/calibration_writer.h
  ${CPP_HEADER_DIR}/controller.h
  ${CPP_HEADER_DIR}/camera.h
  ${CPP_HEADER_DIR}/exception.h
  ${CPP_HEADER_DIR}/image.h
  ${CPP_HEADER_DIR}/light.h
  ${CPP_HEADER_DIR}/light_problem.h
  ${CPP_HEADER_DIR}/light_problem_builder.h
  ${CPP_HEADER_DIR}/light_problem_solver.h
  ${CPP_HEADER_DIR}/linear_response.h
  ${CPP_HEADER_DIR}/poly_response.h
  ${CPP_HEADER_DIR}/response.h
  ${CPP_HEADER_DIR}/response_impl.h
  ${CPP_HEADER_DIR}/response_problem.h
  ${CPP_HEADER_DIR}/response_problem_builder.h
  ${CPP_HEADER_DIR}/response_problem_solver.h
  ${CPP_HEADER_DIR}/vignetting.h
  ${CPP_HEADER_DIR}/vignetting_problem.h
  ${CPP_HEADER_DIR}/vignetting_problem_builder.h
  ${CPP_HEADER_DIR}/vignetting_problem_solver.h
  ${CPP_HEADER_DIR}/visitor.h
)

# define c++ sources

set(CPP_SOURCE_DIR src)

set(CPP_SOURCES
  ${CPP_SOURCE_DIR}/calibration_reader.cpp
  ${CPP_SOURCE_DIR}/calibration_writer.cpp
  ${CPP_SOURCE_DIR}/camera.cpp
  ${CPP_SOURCE_DIR}/light_problem_builder.cpp
  ${CPP_SOURCE_DIR}/light_problem_solver.cpp
  ${CPP_SOURCE_DIR}/response_problem_builder.cpp
  ${CPP_SOURCE_DIR}/response_problem_solver.cpp
  ${CPP_SOURCE_DIR}/vignetting_problem_builder.cpp
  ${CPP_SOURCE_DIR}/vignetting_problem_solver.cpp
)

# add hal camera

find_package(HAL)

if(HAL_LIBRARIES)

  find_package(OpenNI2 REQUIRED)
  list(APPEND REQUIRED_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIRS})
  list(APPEND REQUIRED_LIBRARIES ${OPENNI2_LIBRARIES})
  list(APPEND REQUIRED_INCLUDE_DIRS ${HAL_INCLUDE_DIRS})
  list(APPEND REQUIRED_LIBRARIES ${HAL_LIBRARIES})
  list(APPEND CPP_HEADERS ${CPP_HEADER_DIR}/hal_camera.h)
  list(APPEND CPP_SOURCES ${CPP_SOURCE_DIR}/hal_camera.cpp)

endif()

# build config files

foreach(CPP_HEADER ${CPP_HEADERS})
  string(REPLACE "include/" "" CPP_HEADER ${CPP_HEADER})
  set(PCALIB_INCLUDES "${PCALIB_INCLUDES}\n#include <${CPP_HEADER}>")
endforeach()

foreach(CPP_CONFIG ${CPP_CONFIGS})
  configure_file(${CPP_CONFIG}.in ${CPP_CONFIG} @ONLY)
endforeach()

# build library

add_library(pcalib SHARED ${CPP_SOURCES})
target_link_libraries(pcalib ${REQUIRED_LIBRARIES})
target_include_directories(pcalib PUBLIC ${REQUIRED_INCLUDE_DIRS})
set_property(TARGET pcalib PROPERTY CXX_STANDARD 14)

# build applications

option(PCALIB_BUILD_APPS "Build applications" ON)

if(PCALIB_BUILD_APPS)
  add_subdirectory(apps)
endif()

# build unit tests

option(PCALIB_BUILD_TESTS "Build unit tests" OFF)

if(PCALIB_BUILD_TESTS)
  add_subdirectory(tests)
endif()