cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

# ── MuJoCo discovery ────────────────────────────────────────────────────────
#
# Upstream expects a `mujoco` symlink beside this file, and that symlink is
# gitignored — so a fresh clone has no MuJoCo and fails on a missing header
# rather than with a useful message. Search the usual locations instead, and
# keep honouring the symlink for anyone who already made one.
#
#   -DMUJOCO_DIR=<path>   or   MUJOCO_DIR=<path> in the environment
#
# The layout wanted here is the *release tarball*, which ships `simulate/`
# sources alongside `include/` and `lib/`. A pip-installed mujoco has no
# `simulate/` and will not work.
if(NOT MUJOCO_DIR)
  if(DEFINED ENV{MUJOCO_DIR})
    set(MUJOCO_DIR "$ENV{MUJOCO_DIR}")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include/mujoco/mujoco.h")
    set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mujoco")
  else()
    file(GLOB _mj_candidates "$ENV{HOME}/.mujoco/mujoco-*" "/opt/mujoco-*" "/opt/mujoco")
    list(SORT _mj_candidates)
    list(REVERSE _mj_candidates)  # newest version first
    foreach(_c IN LISTS _mj_candidates)
      if(EXISTS "${_c}/include/mujoco/mujoco.h")
        set(MUJOCO_DIR "${_c}")
        break()
      endif()
    endforeach()
  endif()
endif()

if(NOT MUJOCO_DIR OR NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
  message(FATAL_ERROR
    "MuJoCo not found. Download the release tarball (the one with simulate/ "
    "sources, not the pip package) and either unpack it to ~/.mujoco/, or "
    "pass -DMUJOCO_DIR=<path>.")
endif()
if(NOT EXISTS "${MUJOCO_DIR}/simulate/simulate.cc")
  message(FATAL_ERROR
    "MuJoCo at ${MUJOCO_DIR} has no simulate/ sources — that is the pip "
    "package layout. Use the release tarball.")
endif()
message(STATUS "MuJoCo: ${MUJOCO_DIR}")
set(MUJOCO_DIR "${MUJOCO_DIR}" CACHE PATH "MuJoCo release tarball root" FORCE)

include_directories(
  /usr/local/include/iceoryx/v2.0.2
  ${MUJOCO_DIR}/include
  ${MUJOCO_DIR}/simulate
  src/lodepng
)
link_directories(${MUJOCO_DIR}/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude ${MUJOCO_DIR}/simulate/main.cc
  ${MUJOCO_DIR}/simulate/glfw_*.cc
  ${MUJOCO_DIR}/simulate/platform_*.cc
  ${MUJOCO_DIR}/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

# ── LCM gamepad ─────────────────────────────────────────────────────────────
#
# joystick_type: "lcm" takes the pad off an LCM channel instead of a kernel
# device, which is what makes the controller's FSM scriptable. The binding is
# generated here rather than committed: a generated header that drifts from its
# definition is a silent wire-format mismatch that surfaces as garbage values,
# not as an error.
find_program(LCM_GEN_EXECUTABLE NAMES lcm-gen REQUIRED)
find_path(LCM_INCLUDE_DIR NAMES lcm/lcm-cpp.hpp REQUIRED)
find_library(LCM_LIBRARY NAMES lcm REQUIRED)

set(LCM_SCHEMA ${CMAKE_CURRENT_SOURCE_DIR}/lcm_types/smp_hw_lcm.lcm)
set(LCM_GEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/lcm_generated)
add_custom_command(
  OUTPUT ${LCM_GEN_OUT}/smp_hw_lcm/joy_t.hpp
  COMMAND ${CMAKE_COMMAND} -E make_directory ${LCM_GEN_OUT}
  COMMAND ${LCM_GEN_EXECUTABLE} -x --cpp-std c++11 --cpp-hpath ${LCM_GEN_OUT} ${LCM_SCHEMA}
  DEPENDS ${LCM_SCHEMA}
  COMMENT "lcm-gen: smp_hw_lcm"
  VERBATIM
)
add_custom_target(sim_lcm_types DEPENDS ${LCM_GEN_OUT}/smp_hw_lcm/joy_t.hpp)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc src/lcm_joystick.cc)
add_dependencies(unitree_mujoco sim_lcm_types)
target_include_directories(unitree_mujoco PRIVATE ${LCM_GEN_OUT} ${LCM_INCLUDE_DIR})
target_link_libraries(unitree_mujoco ${LCM_LIBRARY})

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)