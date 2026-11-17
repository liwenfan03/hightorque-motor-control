cmake_minimum_required(VERSION 3.16)
project(hightorque_motor_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(hightorque
  src/protocol.cpp
  src/fdcanusb.cpp
  src/motor.cpp
  src/zero.cpp
  src/control.cpp
  src/pipeline.cpp
)
target_include_directories(hightorque PUBLIC include)
target_compile_options(hightorque PRIVATE -Wall -Wextra)

foreach(example test_motor test_motor_many rate_test motor_tool latency_probe)
  add_executable(${example} examples/${example}.cpp)
  target_link_libraries(${example} PRIVATE hightorque)
  target_compile_options(${example} PRIVATE -Wall -Wextra)
endforeach()

target_compile_definitions(motor_tool PRIVATE HT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")  # finds config/control.conf

enable_testing()
add_executable(protocol_test tests/protocol_test.cpp)
target_link_libraries(protocol_test PRIVATE hightorque)
add_test(NAME protocol_test COMMAND protocol_test)
