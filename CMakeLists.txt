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
    set(PARSER_SRCS src/parser_assembly.cpp
                    src/preprocessor.cpp
                    src/parser_assembly/common.cpp
                    src/parser_assembly/compare.cpp
                    src/parser_assembly/declaration.cpp
                    src/parser_assembly/flowcontrol.cpp
                    src/parser_assembly/floatop.cpp)

    include_directories(${Boost_INCLUDE_DIRS})
    add_executable(nihstro-assemble src/assembler.cpp ${PARSER_SRCS})
    target_link_libraries(nihstro-assemble ${Boost_PROGRAM_OPTIONS_LIBRARY})
    install(TARGETS nihstro-assemble DESTINATION bin)

    # tests
    if(Boost_UNIT_TEST_FRAMEWORK_FOUND)
        add_executable(test-parser src/tests/parser.cpp ${PARSER_SRCS})
        target_compile_definitions(test-parser PUBLIC -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
        target_link_libraries(test-parser ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
        enable_testing()
        add_test(ParserTests test-parser)

        add_executable(test-source-tree-iterator src/tests/source_tree_iterator.cpp)
        target_compile_definitions(test-source-tree-iterator PUBLIC -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
        target_link_libraries(test-source-tree-iterator ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
        add_test(ParserTests test-source-tree-iterator)
    else()
        message(WARNING "Boost testing framework not found => not building assembler tests")
    endif()
else()
    message(WARNING "Boost not found => not building assembler")
endif()
