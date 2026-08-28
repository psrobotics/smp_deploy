# Findlcm.cmake — locate the apt-installed LCM library (liblcm-dev)
# Provides:
#   lcm_FOUND, lcm_INCLUDE_DIRS, lcm_LIBRARIES
#   Imported target: lcm (alias for find_package(lcm) link_libraries(lcm))

find_path(lcm_INCLUDE_DIR
  NAMES lcm/lcm.h
  HINTS /usr/include /usr/local/include
)

find_library(lcm_LIBRARY
  NAMES lcm
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(lcm
  REQUIRED_VARS lcm_LIBRARY lcm_INCLUDE_DIR
)

if(lcm_FOUND)
  set(lcm_INCLUDE_DIRS "${lcm_INCLUDE_DIR}")
  set(lcm_LIBRARIES    "${lcm_LIBRARY}")

  if(NOT TARGET lcm)
    add_library(lcm SHARED IMPORTED)
    set_target_properties(lcm PROPERTIES
      IMPORTED_LOCATION             "${lcm_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${lcm_INCLUDE_DIR}"
    )
  endif()
endif()
