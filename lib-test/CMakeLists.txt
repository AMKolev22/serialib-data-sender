cmake_minimum_required(VERSION 3.30)
project(lib-test)

set(CMAKE_CXX_STANDARD 17)

add_executable(lib-test src/main.cpp
        lib/seriallib.cpp
        lib/seriallib.h)
