# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tutorial1_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tutorial1_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tutorial1_FOUND FALSE)
  elseif(NOT tutorial1_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tutorial1_FOUND FALSE)
  endif()
  return()
endif()
set(_tutorial1_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tutorial1_FIND_QUIETLY)
  message(STATUS "Found tutorial1: 0.0.0 (${tutorial1_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tutorial1' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tutorial1_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tutorial1_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tutorial1_DIR}/${_extra}")
endforeach()
