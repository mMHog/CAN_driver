cmake_minimum_required(VERSION 2.8.3)
project(CAN_driver)
add_compile_options(-std=c++11)

include_directories(
  include include
)
include_directories( "/usr/include/eigen3" )

link_directories(
    lib
)

add_executable(can_test can_test.cpp)

target_link_libraries(can_test
  controlcan
)


