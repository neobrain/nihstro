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
#include <boost/fusion/include/swap.hpp>
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
    ConditionInput,
    (bool, invert)
    (Identifier, identifier)
    (boost::optional<InputSwizzlerMask>, swizzler_mask)
)

BOOST_FUSION_ADAPT_STRUCT(
    Condition,
    (ConditionInput, input1)
    (Instruction::FlowControlType::Op, op)
    (ConditionInput, input2)
)

BOOST_FUSION_ADAPT_STRUCT(
    FlowControlInstruction,
    (OpCode, opcode)
    (std::string, target_label)
    (boost::optional<std::string>, return_label)
    (boost::optional<Condition>, condition)
)

// Manually define a swap() overload for qi::hold to work.
namespace boost {
namespace spirit {
void swap(nihstro::Condition& a, nihstro::Condition& b) {
    boost::fusion::swap(a, b);
}
}
}

template<>
FlowControlParser<ParserIterator>::FlowControlParser(const ParserContext& context)
                : FlowControlParser::base_type(flow_control_instruction),
                  common(context),
                  opcodes_flowcontrol(common.opcodes_flowcontrol),
                  expression(common.expression),
                  identifier(common.identifier),
                  swizzle_mask(common.swizzle_mask),
                  end_of_statement(common.end_of_statement),
                  diagnostics(common.diagnostics) {

        condition_ops.add
                   ( "&&",    ConditionOp::And     )
                   ( "||",    ConditionOp::Or      );

        // Setup rules

        auto blank_rule = qi::omit[ascii::blank];
        auto label_rule = identifier.alias();

        opcode[0] = qi::lexeme[qi::no_case[opcodes_flowcontrol[0]] >> &ascii::space];
        opcode[1] = qi::lexeme[qi::no_case[opcodes_flowcontrol[1]] >> &ascii::space];

        condition_op = qi::lexeme[condition_ops];

        negation = qi::matches[qi::lit("!")];

        condition_input = negation >> identifier >> -(qi::lit('.') > swizzle_mask);

        // May be a condition involving the conditional codes, or a reference to a uniform
        // TODO: Make sure we use qi::hold wherever necessary
        condition = qi::hold[condition_input >> condition_op >> condition_input]
                    | (condition_input >> qi::attr(ConditionOp::JustX) >> qi::attr(ConditionInput{}));

        // if condition
        instr[0] = opcode[0]
                   >> qi::attr("__dummy")  // Dummy label (set indirectly using else,endif, or endloop pseudo-instructions)
                   >> qi::attr(boost::optional<std::string>()) // Dummy return label
                   >> condition;

        // call target_label until return_label if condition
        instr[1] = opcode[1]
                   >> label_rule
                   >> -(qi::no_skip[(blank_rule >> qi::lit("until")) > blank_rule] >> label_rule)
                   >> -(qi::no_skip[(blank_rule >> qi::lit("if")) > blank_rule] >> condition);

        flow_control_instruction %= (instr[0] | instr[1]) > end_of_statement;

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(opcode[0]);
        BOOST_SPIRIT_DEBUG_NODE(opcode[1]);
        BOOST_SPIRIT_DEBUG_NODE(negation);
        BOOST_SPIRIT_DEBUG_NODE(condition_op);
        BOOST_SPIRIT_DEBUG_NODE(condition_input);
        BOOST_SPIRIT_DEBUG_NODE(condition);

        BOOST_SPIRIT_DEBUG_NODE(instr[0]);
        BOOST_SPIRIT_DEBUG_NODE(instr[1]);
        BOOST_SPIRIT_DEBUG_NODE(flow_control_instruction);

        qi::on_error<qi::fail>(flow_control_instruction, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
}
