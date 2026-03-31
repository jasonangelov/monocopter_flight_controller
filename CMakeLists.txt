cmake_minimum_required(VERSION 3.20)

project(monocopter_host_tests LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(BUILD_TESTING)
  find_package(GTest QUIET)

  if(NOT GTest_FOUND)
    include(FetchContent)

    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()
endif()

add_library(
  monocopter_core
  firmware_monocopter/ActuatorMixer.cpp
  firmware_monocopter/ControlSystem.cpp
  firmware_monocopter/FlightController.cpp
)

target_include_directories(monocopter_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/firmware_monocopter)

if(BUILD_TESTING)
  add_executable(
    monocopter_tests
    tests/ActuatorMixerTest.cpp
    tests/ControlSystemTest.cpp
    tests/FlightControllerTest.cpp
  )

  target_link_libraries(monocopter_tests PRIVATE monocopter_core GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(monocopter_tests)
endif()
