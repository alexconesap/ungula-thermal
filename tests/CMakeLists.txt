cmake_minimum_required(VERSION 3.16)
project(ungula_thermal_lib_tests)

set(CMAKE_CXX_STANDARD 17)

set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

set(LIB_THERMAL_SRC ${CMAKE_CURRENT_LIST_DIR}/../src)
set(LIB_GENERIC_SRC   ${CMAKE_CURRENT_LIST_DIR}/../../lib/src)

include_directories(
    ${LIB_THERMAL_SRC}
    ${LIB_GENERIC_SRC}
    ${CMAKE_CURRENT_LIST_DIR}/ArduinoStub
)

# lib_thermal source code has no logger calls — no emblogx or lib dependency
# needed for the test build.
add_compile_definitions(
)

add_executable(test_ungula_thermal
    test_pid_controller.cpp
    test_pid_comprehensive.cpp
    test_temp_sensor.cpp
    test_heater_channel.cpp
    test_fan_control.cpp
    ${LIB_THERMAL_SRC}/ungula/thermal/pid_controller.cpp
    ${LIB_THERMAL_SRC}/ungula/thermal/heater_control.cpp
    ${LIB_THERMAL_SRC}/ungula/thermal/temp_sensor.cpp
    ${LIB_THERMAL_SRC}/ungula/thermal/fan_control.cpp
)
target_link_libraries(test_ungula_thermal gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_ungula_thermal)
