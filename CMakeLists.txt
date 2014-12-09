cmake_minimum_required(VERSION 2.6)

project(nihstro)

find_package(Boost)

add_definitions(-std=c++11)

include_directories(include)

add_executable(nihstro-disassemble src/disassembler.cpp)
install(TARGETS nihstro-disassemble DESTINATION bin)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    add_executable(nihstro-assemble src/assembler.cpp src/parser_assembly.cpp)
    install(TARGETS nihstro-assemble DESTINATION bin)
else()
    message(WARNING "Boost not found => not building assembler")
endif()
