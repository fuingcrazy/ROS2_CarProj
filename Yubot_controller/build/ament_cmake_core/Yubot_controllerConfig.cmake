# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Yubot_controller_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Yubot_controller_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Yubot_controller_FOUND FALSE)
  elseif(NOT Yubot_controller_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Yubot_controller_FOUND FALSE)
  endif()
  return()
endif()
set(_Yubot_controller_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Yubot_controller_FIND_QUIETLY)
  message(STATUS "Found Yubot_controller: 0.0.0 (${Yubot_controller_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Yubot_controller' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Yubot_controller_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Yubot_controller_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Yubot_controller_DIR}/${_extra}")
endforeach()
