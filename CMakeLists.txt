cmake_minimum_required(VERSION 3.16)
project(hps6axis_sdk VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hps6axis src/hps6axis.cpp)
target_include_directories(hps6axis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(hps6axis PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hps6axis PRIVATE pthread)

add_executable(hps6axis_read examples/read_sensor.cpp)
target_link_libraries(hps6axis_read PRIVATE hps6axis)

add_executable(hps6axis_once examples/read_once.cpp)
target_link_libraries(hps6axis_once PRIVATE hps6axis)

add_executable(hps6axis_stop examples/stop_continuous.cpp)
target_link_libraries(hps6axis_stop PRIVATE hps6axis)

add_executable(hps6axis_server examples/hps6axis_server.cpp)
target_link_libraries(hps6axis_server PRIVATE hps6axis)

include(CTest)
if(BUILD_TESTING)
  add_executable(hps6axis_protocol_test tests/test_protocol.cpp)
  target_link_libraries(hps6axis_protocol_test PRIVATE hps6axis)
  add_test(NAME hps6axis_protocol_test COMMAND hps6axis_protocol_test)
endif()

install(TARGETS hps6axis EXPORT hps6axisTargets)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT hps6axisTargets DESTINATION lib/cmake/hps6axis)
