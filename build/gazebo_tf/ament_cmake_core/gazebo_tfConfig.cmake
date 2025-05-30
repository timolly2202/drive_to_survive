# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gazebo_tf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gazebo_tf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gazebo_tf_FOUND FALSE)
  elseif(NOT gazebo_tf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gazebo_tf_FOUND FALSE)
  endif()
  return()
endif()
set(_gazebo_tf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gazebo_tf_FIND_QUIETLY)
  message(STATUS "Found gazebo_tf: 0.2.0 (${gazebo_tf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gazebo_tf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gazebo_tf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gazebo_tf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gazebo_tf_DIR}/${_extra}")
endforeach()
