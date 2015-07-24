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

/*BOOST_FUSION_ADAPT_STRUCT(
    IntegerWithSign,
    (int, sign)
    (unsigned, value)
)
*/
BOOST_FUSION_ADAPT_STRUCT(
    CompareInstruction,
    (OpCode, opcode)
    (std::vector<Expression>, arguments)
    (std::vector<Instruction::Common::CompareOpType::Op>, ops)
)

template<>
CompareParser<ParserIterator>::CompareParser(const ParserContext& context)
                : CompareParser::base_type(instruction),
                  common(context),
                  opcodes_compare(common.opcodes_compare),
                  expression(common.expression),
                  end_of_statement(common.end_of_statement),
                  diagnostics(common.diagnostics) {

        // TODO: Will this properly match >= ?
        compare_ops.add
                       ( "==", CompareOp::Equal )
                       ( "!=", CompareOp::NotEqual )
                       ( "<", CompareOp::LessThan )
                       ( "<=", CompareOp::LessEqual )
                       ( ">", CompareOp::GreaterThan )
                       ( ">=", CompareOp::GreaterEqual );

        // Setup rules

        auto comma_rule = qi::lit(',');

        opcode = qi::no_case[qi::lexeme[opcodes_compare >> &ascii::space]];
        compare_op = qi::lexeme[compare_ops];

        // cmp src1, src2, op1, op2
        // TODO: Also allow "cmp src1 op1 src2, src1 op2 src2"
        two_ops = compare_op > comma_rule > compare_op;
        two_expressions = expression > comma_rule > expression;
        instr[0] = opcode > two_expressions > comma_rule > two_ops;

        instruction = instr[0] > end_of_statement;

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(instr[0]);
        BOOST_SPIRIT_DEBUG_NODE(instruction);

        qi::on_error<qi::fail>(instruction, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
}
