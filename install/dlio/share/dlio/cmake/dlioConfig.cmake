# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dlio_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dlio_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dlio_FOUND FALSE)
  elseif(NOT dlio_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dlio_FOUND FALSE)
  endif()
  return()
endif()
set(_dlio_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dlio_FIND_QUIETLY)
  message(STATUS "Found dlio: 0.0.0 (${dlio_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dlio' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dlio_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dlio_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dlio_DIR}/${_extra}")
endforeach()
