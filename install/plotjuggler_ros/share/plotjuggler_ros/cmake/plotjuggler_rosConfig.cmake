# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_plotjuggler_ros_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED plotjuggler_ros_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(plotjuggler_ros_FOUND FALSE)
  elseif(NOT plotjuggler_ros_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(plotjuggler_ros_FOUND FALSE)
  endif()
  return()
endif()
set(_plotjuggler_ros_CONFIG_INCLUDED TRUE)

# output package information
if(NOT plotjuggler_ros_FIND_QUIETLY)
  message(STATUS "Found plotjuggler_ros: 2.3.1 (${plotjuggler_ros_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'plotjuggler_ros' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${plotjuggler_ros_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(plotjuggler_ros_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${plotjuggler_ros_DIR}/${_extra}")
endforeach()
