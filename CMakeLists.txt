cmake_minimum_required(VERSION 2.6)

project(nihstro)

add_definitions(-std=c++11)

add_executable(nihstro-disassemble main.cpp)
add_executable(nihstro-assemble assembler.cpp)
