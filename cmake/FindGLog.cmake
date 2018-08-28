# - Try to find glog
# Once done, this will define
#
#  GLog_FOUND - system has glog
#  GLog_INCLUDE_DIRS - the glog include directories
#  GLog_LIBRARIES - link these to use glog

# Find header and lib
find_path(GLog_INCLUDE_DIR NAMES glog/logging.h)
find_library(GLog_LIBRARIES NAMES glog)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLog DEFAULT_MSG GLog_INCLUDE_DIR
  GLog_LIBRARIES)
set(Glog_INCLUDE_DIRS ${GLog_INCLUDE_DIRS})
set(Glog_LIBRARIES ${GLog_LIBRARIES})
