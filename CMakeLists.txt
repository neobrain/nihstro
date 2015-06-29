cmake_minimum_required(VERSION 2.6)

project(nihstro)

find_package(Boost COMPONENTS program_options unit_test_framework)

add_definitions(-std=c++11)

include_directories(include)

add_executable(nihstro-disassemble src/disassembler.cpp src/parser_shbin.cpp)
install(TARGETS nihstro-disassemble DESTINATION bin)

# TODO: Re-enable
# add_subdirectory(examples/inline_assembler/simple)

# TODO: What if program_options was found but not unit_test_framework?
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    add_executable(nihstro-assemble src/assembler.cpp src/parser_assembly.cpp)
    target_link_libraries(nihstro-assemble ${Boost_PROGRAM_OPTIONS_LIBRARY})
    install(TARGETS nihstro-assemble DESTINATION bin)

    # tests
    if(Boost_UNIT_TEST_FRAMEWORK_FOUND)
        add_executable(test-parser src/tests/parser.cpp src/parser_assembly.cpp)
        target_compile_definitions(test-parser PUBLIC -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
        target_link_libraries(test-parser ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
        enable_testing()
        add_test(ParserTests test-parser)
    else()
        message(WARNING "Boost testing framework not found => not building assembler tests")
    endif()
else()
    message(WARNING "Boost not found => not building assembler")
endif()
