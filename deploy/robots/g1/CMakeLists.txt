cmake_minimum_required(VERSION 3.12)
project(g1_controller)

set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(lcm REQUIRED)

set(REPO_ROOT "${PROJECT_SOURCE_DIR}/../../..")

# Not vendored: 20-40 MB of prebuilt binaries per arch would cost every clone.
# scripts/fetch_onnxruntime.sh pulls the official CPU release for this host; the
# policies are 854 KB and infer well inside the 20 ms control period on a CPU.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ORT_ARCH "x64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ORT_ARCH "aarch64")
else()
    message(FATAL_ERROR "Unsupported processor '${CMAKE_SYSTEM_PROCESSOR}' — "
                        "add an ONNX Runtime release for it in scripts/fetch_onnxruntime.sh")
endif()
set(ONNXRUNTIME_DIR "${REPO_ROOT}/deploy/thirdparty/onnxruntime-linux-${ORT_ARCH}-1.22.0")
if(NOT EXISTS "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0")
    message(FATAL_ERROR "ONNX Runtime not found at ${ONNXRUNTIME_DIR}\n"
                        "  Run: scripts/fetch_onnxruntime.sh")
endif()

# Bake the path in rather than requiring LD_LIBRARY_PATH at run time.
set(CMAKE_BUILD_RPATH "${ONNXRUNTIME_DIR}/lib")
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib")

# Generated at build time, never committed: a header that drifts from its
# definition is a silent wire-format mismatch, seen as garbage floats.
set(LCM_GEN_DIR "${CMAKE_BINARY_DIR}/lcm_generated")
set(LCM_DEF "${REPO_ROOT}/lcm_types/smp_hw_lcm.lcm")
find_program(LCM_GEN_EXECUTABLE lcm-gen REQUIRED)
add_custom_command(
    OUTPUT "${LCM_GEN_DIR}/smp_hw_lcm/robot_state_t.hpp"
           "${LCM_GEN_DIR}/smp_hw_lcm/cmd_t.hpp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LCM_GEN_DIR}"
    COMMAND ${LCM_GEN_EXECUTABLE} -x --cpp-hpath "${LCM_GEN_DIR}" "${LCM_DEF}"
    DEPENDS "${LCM_DEF}"
    COMMENT "lcm-gen: smp_hw_lcm"
    VERBATIM
)
add_custom_target(lcm_headers DEPENDS "${LCM_GEN_DIR}/smp_hw_lcm/robot_state_t.hpp")

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${ONNXRUNTIME_DIR}/include
  ${LCM_GEN_DIR}
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  lcm
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
add_dependencies(${PROJECT_NAME}_lib lcm_headers)
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)
add_dependencies(g1_ctrl lcm_headers)

# Measures what the scheduler gives you, before you blame the policy.
add_executable(rt_bench rt_bench.cpp)
