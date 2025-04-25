# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pepper_autonomy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pepper_autonomy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pepper_autonomy_FOUND FALSE)
  elseif(NOT pepper_autonomy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pepper_autonomy_FOUND FALSE)
  endif()
  return()
endif()
set(_pepper_autonomy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pepper_autonomy_FIND_QUIETLY)
  message(STATUS "Found pepper_autonomy: 0.0.0 (${pepper_autonomy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pepper_autonomy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pepper_autonomy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pepper_autonomy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pepper_autonomy_DIR}/${_extra}")
endforeach()
