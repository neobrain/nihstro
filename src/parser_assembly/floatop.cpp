// Copyright 2014 Tony Wasserka
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the owner nor the names of its contributors may
//       be used to endorse or promote products derived from this software
//       without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


// Enable this for detailed XML overview of parser results
// #define BOOST_SPIRIT_DEBUG

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi.hpp>

#include "nihstro/parser_assembly.h"
#include "nihstro/parser_assembly_private.h"

#include "nihstro/shader_binary.h"
#include "nihstro/shader_bytecode.h"

namespace spirit = boost::spirit;
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::qi::ascii;
namespace phoenix = boost::phoenix;

using spirit::_1;
using spirit::_2;
using spirit::_3;
using spirit::_4;

using namespace nihstro;

// Adapt parser data structures for use with boost::spirit

BOOST_FUSION_ADAPT_STRUCT(
    StatementInstruction,
    (OpCode, opcode)
    (std::vector<Expression>, expressions)
)

template<>
FloatOpParser<ParserIterator>::FloatOpParser(const ParserContext& context)
                : FloatOpParser::base_type(float_instruction),
                  common(context),
                  opcodes_float(common.opcodes_float),
                  expression(common.expression),
                  end_of_statement(common.end_of_statement),
                  diagnostics(common.diagnostics) {

        // Setup rules

        auto comma_rule = qi::lit(',');

        for (int i = 0; i < 4; ++i) {
            // Make sure that a mnemonic is always followed by a space (such that e.g. "addbla" fails to match)
            opcode[i] = qi::no_case[qi::lexeme[opcodes_float[i] >> &ascii::space]];
        }

        // chain of arguments for each group of opcodes
        expression_chain[0] = expression;
        for (int i = 1; i < 4; ++i) {
            expression_chain[i] = expression_chain[i - 1] >> comma_rule > expression;
        }

        // e.g. "add o1, t2, t5"
        float_instr[0] = opcode[0] > expression_chain[0];
        float_instr[1] = opcode[1] > expression_chain[1];
        float_instr[2] = opcode[2] > expression_chain[2];
        float_instr[3] = opcode[3] > expression_chain[3];

        float_instruction %= (float_instr[0] | float_instr[1] | float_instr[2] | float_instr[3]) > end_of_statement;

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(opcode[0]);
        BOOST_SPIRIT_DEBUG_NODE(opcode[1]);
        BOOST_SPIRIT_DEBUG_NODE(opcode[2]);
        BOOST_SPIRIT_DEBUG_NODE(opcode[3]);

        BOOST_SPIRIT_DEBUG_NODE(expression_chain[0]);
        BOOST_SPIRIT_DEBUG_NODE(expression_chain[1]);
        BOOST_SPIRIT_DEBUG_NODE(expression_chain[2]);
        BOOST_SPIRIT_DEBUG_NODE(expression_chain[3]);

        BOOST_SPIRIT_DEBUG_NODE(float_instr[0]);
        BOOST_SPIRIT_DEBUG_NODE(float_instr[1]);
        BOOST_SPIRIT_DEBUG_NODE(float_instr[2]);
        BOOST_SPIRIT_DEBUG_NODE(float_instr[3]);
        BOOST_SPIRIT_DEBUG_NODE(float_instruction);

        diagnostics.Add(expression_chain[0].name(), "one argument");
        diagnostics.Add(expression_chain[1].name(), "two arguments");
        diagnostics.Add(expression_chain[2].name(), "three arguments");
        diagnostics.Add(expression_chain[3].name(), "four arguments");

        qi::on_error<qi::fail>(float_instruction, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
}
