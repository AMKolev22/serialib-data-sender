cmake_minimum_required(VERSION 3.30)
project(lib-test)

set(CMAKE_CXX_STANDARD 17)

add_executable(lib-test 
    src/main.cpp
    lib/seriallib.cpp
    lib/seriallib.h
    src/server.h
    src/server.cpp
)

include_directories(lib/)
include_directories(lib/include/)

find_package(OpenSSL REQUIRED) # Finds OpenSSL library
include_directories(${OPENSSL_INCLUDE_DIR}) # Includes OpenSSL headers
target_link_libraries(lib-test OpenSSL::SSL OpenSSL::Crypto) # Links OpenSSL libraries
