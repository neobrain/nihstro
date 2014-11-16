cmake_minimum_required(VERSION 2.6)

project(nihstro)

find_package(Boost)

add_definitions(-std=c++11)

add_executable(nihstro-disassemble src/disassembler.cpp)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    add_executable(nihstro-assemble src/assembler.cpp src/parser_assembly.cpp)
else()
    message(WARNING "Boost not found => not building assembler")
endif()
