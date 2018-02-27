cmake_minimum_required(VERSION 2.8.3)
project(ctoc++)

SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)

include_directories("${PROJECT_BINARY_DIR}")

add_executable(ctoc++ src/main.cpp src/test.cpp)
