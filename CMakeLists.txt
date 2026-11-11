# Basalt-Win: native Windows C++ port of Basalt stereo-inertial odometry.
# Dependencies: Eigen3, OpenCV (core, imgproc, imgcodecs, features2d), OpenMP (optional).
cmake_minimum_required(VERSION 3.20)

# --- Windows: find vcpkg automatically if no toolchain file was given ---------
if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  # VCPKG_ROOT first, then the private copy made by scripts/setup_vcpkg.ps1,
  # then common install locations.
  set(_vcpkg_candidates "$ENV{VCPKG_ROOT}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/vcpkg"
                        "C:/vcpkg" "C:/src/vcpkg" "C:/tools/vcpkg" "$ENV{USERPROFILE}/vcpkg")
  foreach(_c ${_vcpkg_candidates})
    if(_c AND EXISTS "${_c}/scripts/buildsystems/vcpkg.cmake")
      set(CMAKE_TOOLCHAIN_FILE "${_c}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "vcpkg toolchain")
      message(STATUS "Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
      break()
    endif()
  endforeach()
endif()
if(WIN32 AND NOT DEFINED VCPKG_TARGET_TRIPLET)
  set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "vcpkg triplet")
endif()

project(basalt_win LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d)
find_package(OpenMP)

add_library(bw_core STATIC
  src/config.cpp
  src/dataset.cpp
  src/optical_flow.cpp
  src/imu.cpp
  src/vio_estimator.cpp
  src/eval.cpp
  src/report.cpp
)
target_include_directories(bw_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bw_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
target_compile_definitions(bw_core PUBLIC _USE_MATH_DEFINES NOMINMAX
  BW_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
if(OpenMP_CXX_FOUND)
  target_link_libraries(bw_core PUBLIC OpenMP::OpenMP_CXX)
endif()
if(MSVC)
  target_compile_options(bw_core PUBLIC /W3 /bigobj /permissive- /utf-8 /wd4996
    $<$<CONFIG:Release>:/O2 /Ob2 /DNDEBUG>)
else()
  target_compile_options(bw_core PUBLIC -Wall -O3)
endif()

add_executable(bw_vio src/main_vio.cpp)
target_link_libraries(bw_vio PRIVATE bw_core)

add_executable(bw_eval src/main_eval.cpp)
target_link_libraries(bw_eval PRIVATE bw_core)

add_executable(bw_tests src/tests.cpp)
target_link_libraries(bw_tests PRIVATE bw_core)

enable_testing()
add_test(NAME jacobians COMMAND bw_tests)
