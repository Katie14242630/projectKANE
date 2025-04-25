# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_book_database_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED book_database_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(book_database_FOUND FALSE)
  elseif(NOT book_database_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(book_database_FOUND FALSE)
  endif()
  return()
endif()
set(_book_database_CONFIG_INCLUDED TRUE)

# output package information
if(NOT book_database_FIND_QUIETLY)
  message(STATUS "Found book_database: 0.1.0 (${book_database_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'book_database' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${book_database_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(book_database_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${book_database_DIR}/${_extra}")
endforeach()
