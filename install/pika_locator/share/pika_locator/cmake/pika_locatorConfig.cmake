# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pika_locator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pika_locator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pika_locator_FOUND FALSE)
  elseif(NOT pika_locator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pika_locator_FOUND FALSE)
  endif()
  return()
endif()
set(_pika_locator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pika_locator_FIND_QUIETLY)
  message(STATUS "Found pika_locator: 0.0.1 (${pika_locator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pika_locator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pika_locator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pika_locator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pika_locator_DIR}/${_extra}")
endforeach()
