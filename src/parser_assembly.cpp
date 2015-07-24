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
    SetEmitInstruction::Flags,
    (boost::optional<bool>, primitive_flag)
    (boost::optional<bool>, invert_flag)
)

BOOST_FUSION_ADAPT_STRUCT(
    SetEmitInstruction,
    (OpCode, opcode)
    (unsigned, vertex_id)
    (SetEmitInstruction::Flags, flags)
)

phoenix::function<ErrorHandler> error_handler;

template<typename Iterator, bool require_end_of_line>
TrivialOpParser<Iterator, require_end_of_line>::TrivialOpParser(const ParserContext& context)
                : TrivialOpParser::base_type(trivial_instruction),
                  common(context),
                  opcodes_trivial(common.opcodes_trivial),
                  opcodes_compare(common.opcodes_compare),
                  opcodes_float(common.opcodes_float),
                  opcodes_flowcontrol(common.opcodes_flowcontrol),
                  end_of_statement(common.end_of_statement),
                  diagnostics(common.diagnostics) {

        // Setup rules
        if (require_end_of_line) {
            opcode = qi::no_case[qi::lexeme[opcodes_trivial >> &ascii::space]];
            trivial_instruction = opcode > end_of_statement;
        } else {
            opcode = qi::no_case[qi::lexeme[opcodes_trivial | opcodes_compare | opcodes_float[0]
                                            | opcodes_float[1] | opcodes_float[2] | opcodes_float[3]
                                            | opcodes_flowcontrol[0] | opcodes_flowcontrol[1] >> &ascii::space]];
            trivial_instruction = opcode;
        }

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(opcode);
        BOOST_SPIRIT_DEBUG_NODE(trivial_instruction);

        qi::on_error<qi::fail>(trivial_instruction, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
}

template<typename Iterator>
SetEmitParser<Iterator>::SetEmitParser(const ParserContext& context)
                : SetEmitParser::base_type(setemit_instruction),
                  common(context),
                  opcodes_setemit(common.opcodes_setemit),
                  end_of_statement(common.end_of_statement),
                  diagnostics(common.diagnostics) {

        // Setup rules

        auto comma_rule = qi::lit(',');

        opcode = qi::lexeme[qi::no_case[opcodes_setemit] >> &ascii::space];

        vertex_id = qi::uint_;
        prim_flag = qi::lit("prim") >> &(!ascii::alnum) >> qi::attr(true);
        inv_flag = qi::lit("inv") >> &(!ascii::alnum) >> qi::attr(true);
        flags = ((comma_rule >> prim_flag) ^ (comma_rule >> inv_flag));

        setemit_instruction = ((opcode >> vertex_id) >> (flags | qi::attr(SetEmitInstruction::Flags{}))) > end_of_statement;

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(opcode);
        BOOST_SPIRIT_DEBUG_NODE(vertex_id);
        BOOST_SPIRIT_DEBUG_NODE(prim_flag);
        BOOST_SPIRIT_DEBUG_NODE(inv_flag);
        BOOST_SPIRIT_DEBUG_NODE(flags);
        BOOST_SPIRIT_DEBUG_NODE(setemit_instruction);

        qi::on_error<qi::fail>(setemit_instruction, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
}

template<typename Iterator>
LabelParser<Iterator>::LabelParser(const ParserContext& context)
                : LabelParser::base_type(label), common(context),
                  end_of_statement(common.end_of_statement),
                  identifier(common.identifier),
                  diagnostics(common.diagnostics) {

        label = identifier >> qi::lit(':') > end_of_statement;

        BOOST_SPIRIT_DEBUG_NODE(label);

        qi::on_error<qi::fail>(label, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
}
template struct LabelParser<ParserIterator>;


struct Parser::ParserImpl {
    using Iterator = SourceTreeIterator;

    ParserImpl(const ParserContext& context) : label(context), plain_instruction(context),
                                               simple_instruction(context), instruction(context),
                                               compare(context), flow_control(context),
                                               setemit(context), declaration(context) {
    }

    unsigned Skip(Iterator& begin, Iterator end) {
        unsigned lines_skipped = 0;
        do {
            parse(begin, end, skipper);
            lines_skipped++;
        } while (boost::spirit::qi::parse(begin, end, boost::spirit::qi::eol));

        return --lines_skipped;
    }

    void SkipSingleLine(Iterator& begin, Iterator end) {
        qi::parse(begin, end, *(qi::char_ - (qi::eol | qi::eoi)) >> (qi::eol | qi::eoi));
    }

    bool ParseLabel(Iterator& begin, Iterator end, StatementLabel* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, label, skipper, *content);
    }

    bool ParseOpCode(Iterator& begin, Iterator end, OpCode* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, plain_instruction, skipper, *content);
    }

    bool ParseSimpleInstruction(Iterator& begin, Iterator end, OpCode* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, simple_instruction, skipper, *content);
    }

    bool ParseFloatOp(Iterator& begin, Iterator end, FloatOpInstruction* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, instruction, skipper, *content);
    }

    bool ParseCompare(Iterator& begin, Iterator end, CompareInstruction* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, compare, skipper, *content);
    }

    bool ParseFlowControl(Iterator& begin, Iterator end, FlowControlInstruction* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, flow_control, skipper, *content);
    }

    bool ParseSetEmit(Iterator& begin, Iterator end, SetEmitInstruction* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, setemit, skipper, *content);
    }

    bool ParseDeclaration(Iterator& begin, Iterator end, StatementDeclaration* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, declaration, skipper, *content);
    }

private:
    AssemblySkipper<Iterator>   skipper;

    LabelParser<Iterator>       label;
    TrivialOpParser<Iterator, false> plain_instruction;
    TrivialOpParser<Iterator, true>  simple_instruction;
    FloatOpParser<Iterator>     instruction;
    CompareParser<Iterator>     compare;
    FlowControlParser<Iterator> flow_control;
    SetEmitParser<Iterator> setemit;
    DeclarationParser<Iterator> declaration;
};



Parser::Parser(const ParserContext& context) : impl(new ParserImpl(context)) {
};

Parser::~Parser() {
}

unsigned Parser::Skip(Iterator& begin, Iterator end) {
    return impl->Skip(begin, end);
}

void Parser::SkipSingleLine(Iterator& begin, Iterator end) {
    impl->SkipSingleLine(begin, end);
}

bool Parser::ParseLabel(Iterator& begin, Iterator end, StatementLabel* label) {
    return impl->ParseLabel(begin, end, label);
}

bool Parser::ParseOpCode(Iterator& begin, Iterator end, OpCode* opcode) {
    return impl->ParseOpCode(begin, end, opcode);
}

bool Parser::ParseSimpleInstruction(Iterator& begin, Iterator end, OpCode* opcode) {
    return impl->ParseSimpleInstruction(begin, end, opcode);
}

bool Parser::ParseFloatOp(Iterator& begin, Iterator end, FloatOpInstruction* instruction) {
    return impl->ParseFloatOp(begin, end, instruction);
}

bool Parser::ParseCompare(Iterator& begin, Iterator end, CompareInstruction* content) {
    return impl->ParseCompare(begin, end, content);
}

bool Parser::ParseFlowControl(Iterator& begin, Iterator end, FlowControlInstruction* content) {
    return impl->ParseFlowControl(begin, end, content);
}

bool Parser::ParseSetEmit(Iterator& begin, Iterator end, SetEmitInstruction* content) {
    return impl->ParseSetEmit(begin, end, content);
}

bool Parser::ParseDeclaration(Iterator& begin, Iterator end, StatementDeclaration* declaration) {
    return impl->ParseDeclaration(begin, end, declaration);
}
