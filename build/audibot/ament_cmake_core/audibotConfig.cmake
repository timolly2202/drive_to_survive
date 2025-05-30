# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_audibot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED audibot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(audibot_FOUND FALSE)
  elseif(NOT audibot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(audibot_FOUND FALSE)
  endif()
  return()
endif()
set(_audibot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT audibot_FIND_QUIETLY)
  message(STATUS "Found audibot: 0.2.1 (${audibot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'audibot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${audibot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(audibot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${audibot_DIR}/${_extra}")
endforeach()
