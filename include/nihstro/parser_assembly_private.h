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

#pragma once

// Enable this for detailed XML overview of parser results
// #define BOOST_SPIRIT_DEBUG

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi.hpp>

#include "nihstro/parser_assembly.h"
#include "nihstro/source_tree.h"

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

// Adapt common parser data structures for use with boost::spirit

BOOST_FUSION_ADAPT_STRUCT(
    Expression::SignedIdentifier,
    (boost::optional<Sign>, sign)
    (Identifier, identifier)
)

BOOST_FUSION_ADAPT_STRUCT(
    Expression,
    (Expression::SignedIdentifier, signed_identifier)
    (boost::optional<IndexExpression>, index)
    (std::vector<InputSwizzlerMask>, swizzle_masks)
)

class Diagnostics
{
public:
    // Ass a new diagnostic message corresponding to the specified rule tag
    void Add(const std::string& tag, const char* diagnostic) {
        entries[tag] = diagnostic;
    }

    // Lookup the diagnostic of the specified rule tag and return it (or nullptr if it can't be found)
    const char* operator [](const char* tag) const {
        auto it = entries.find(tag);
        if (it == entries.end())
            return nullptr;
        else
            return it->second;
    }

private:
    std::map<std::string, const char*> entries;
};

struct ErrorHandler
{
    template <class, class, class, class, class>
    struct result { typedef void type; };

    template <class D, class B, class E, class W, class I>
    void operator ()(const D& diagnostics, B begin, E end, W where, const I& info) const
    {
        const spirit::utf8_string& tag(info.tag);
        const char* const what(tag.c_str());
        const char* diagnostic(diagnostics[what]);
        std::string scratch;
        if (!diagnostic) {
            scratch.reserve(25 + tag.length());
            scratch = "Expected ";
            scratch += tag;
            diagnostic = scratch.c_str();
        }

        auto newline_iterator = std::find(begin, end, '\n');

        std::stringstream err;
        err << diagnostic << std::endl
            << std::string(4, ' ') << std::string(begin, newline_iterator) << std::endl
            << std::string(4 + std::distance(begin, where), ' ') << '^' << std::endl;
        throw err.str();
    }
};
extern phoenix::function<ErrorHandler> error_handler;

template<typename Iterator>
struct AssemblySkipper : public qi::grammar<Iterator> {

    AssemblySkipper() : AssemblySkipper::base_type(skip) {
        comments = (qi::lit("//") | '#' | ';') >> *(qi::char_ - qi::eol);

        skip = +(comments | ascii::blank);
    }

    qi::rule<Iterator> comments;
    qi::rule<Iterator> skip;
};

namespace std {

static std::ostream& operator<<(std::ostream& os, const OpCode& opcode) {
    // TODO: Should print actual opcode here..
    return os << static_cast<uint32_t>(static_cast<OpCode::Id>(opcode));
}

}

template<typename Iterator>
struct CommonRules {
    using Skipper = AssemblySkipper<Iterator>;

    CommonRules(const ParserContext& context);

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator,                            Skipper> peek_identifier;

    // Building blocks
    qi::rule<Iterator, std::string(),             Skipper> identifier;
    qi::rule<Iterator, Expression(),              Skipper> expression;
    qi::rule<Iterator,                            Skipper> end_of_statement;

    qi::symbols<char, OpCode> opcodes_trivial;
    qi::symbols<char, OpCode> opcodes_compare;
    std::array<qi::symbols<char, OpCode>, 4> opcodes_float; // indexed by number of arguments
    std::array<qi::symbols<char, OpCode>, 2> opcodes_flowcontrol;
    qi::symbols<char, OpCode> opcodes_setemit;

    qi::symbols<char, int>    signs;

    qi::symbols<char, InputSwizzlerMask::Component>        swizzlers;
    qi::rule<Iterator, InputSwizzlerMask(),       Skipper> swizzle_mask;

    Diagnostics diagnostics;

private:
    qi::rule<Iterator, IndexExpression(),                             Skipper> index_expression;
    qi::rule<Iterator, boost::variant<IntegerWithSign, Identifier>(), Skipper> index_expression_first_term;
    qi::rule<Iterator, boost::variant<IntegerWithSign, Identifier>(), Skipper> index_expression_following_terms;

    // Empty rule
    qi::rule<Iterator,                            Skipper> opening_bracket;
    qi::rule<Iterator,                            Skipper> closing_bracket;
    qi::rule<Iterator, IntegerWithSign(),         Skipper> sign_with_uint;
    qi::rule<Iterator, unsigned int(),            Skipper> uint_after_sign;
};

template<typename Iterator, bool require_end_of_line>
struct TrivialOpParser : qi::grammar<Iterator, OpCode(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    TrivialOpParser(const ParserContext& context);

    CommonRules<Iterator> common;

    qi::symbols<char, OpCode>& opcodes_trivial;
    qi::symbols<char, OpCode>& opcodes_compare;
    std::array<qi::symbols<char, OpCode>, 4>& opcodes_float; // indexed by number of arguments
    std::array<qi::symbols<char, OpCode>, 2>& opcodes_flowcontrol;

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator, OpCode(), Skipper> opcode;

    // Compounds
    qi::rule<Iterator, OpCode(), Skipper> trivial_instruction;
    qi::rule<Iterator,           Skipper>& end_of_statement;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct FloatOpParser : qi::grammar<Iterator, FloatOpInstruction(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    FloatOpParser(const ParserContext& context);

    CommonRules<Iterator> common;

    std::array<qi::symbols<char, OpCode>, 4>& opcodes_float;

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator, OpCode(),                  Skipper>  opcode[4];

    // Building blocks
    qi::rule<Iterator, Expression(),              Skipper>& expression;
    qi::rule<Iterator, std::vector<Expression>(), Skipper>  expression_chain[4]; // sequence of instruction arguments
    qi::rule<Iterator,                            Skipper>& end_of_statement;

    // Compounds
    qi::rule<Iterator, FloatOpInstruction(),    Skipper>    float_instr[4];
    qi::rule<Iterator, FloatOpInstruction(),    Skipper>    float_instruction;

    // Utility
    qi::rule<Iterator,                            Skipper>  not_comma;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct CompareParser : qi::grammar<Iterator, CompareInstruction(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;
    using CompareOp = Instruction::Common::CompareOpType;
    using CompareOpEnum = CompareOp::Op;

    CompareParser(const ParserContext& context);

    CommonRules<Iterator> common;

    qi::symbols<char, OpCode>&                    opcodes_compare;
    qi::symbols<char, CompareOpEnum>              compare_ops;

    // Rule-ified symbols, which can be assigned debug names
    qi::rule<Iterator, OpCode(),                  Skipper>    opcode;
    qi::rule<Iterator, CompareOpEnum(),           Skipper>    compare_op;
    qi::rule<Iterator, std::vector<CompareOpEnum>(), Skipper> two_ops;

    // Building blocks
    qi::rule<Iterator, Expression(),              Skipper>& expression;
    qi::rule<Iterator, std::vector<Expression>(), Skipper>  two_expressions;
    qi::rule<Iterator,                            Skipper>& end_of_statement;

    // Compounds
    qi::rule<Iterator, CompareInstruction(),    Skipper> instr[1];
    qi::rule<Iterator, CompareInstruction(),    Skipper> instruction;

    // Utility
    qi::rule<Iterator,                            Skipper> not_comma;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct FlowControlParser : qi::grammar<Iterator, FlowControlInstruction(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;
    using ConditionOp = Instruction::FlowControlType;
    using ConditionOpEnum = Instruction::FlowControlType::Op;

    FlowControlParser(const ParserContext& context);

    CommonRules<Iterator> common;

    std::array<qi::symbols<char, OpCode>, 2>&     opcodes_flowcontrol;
    qi::symbols<char, ConditionOpEnum>            condition_ops;

    // Rule-ified symbols, which can be assigned debug names
    qi::rule<Iterator, OpCode(),                  Skipper> opcode[2];
    qi::rule<Iterator, ConditionOpEnum(),         Skipper> condition_op;

    // Building blocks
    qi::rule<Iterator, Expression(),              Skipper>& expression;
    qi::rule<Iterator, std::string(),             Skipper>& identifier;
    qi::rule<Iterator, InputSwizzlerMask(),       Skipper>& swizzle_mask;
    qi::rule<Iterator, ConditionInput(),          Skipper>  condition_input;
    qi::rule<Iterator, Condition(),               Skipper>  condition;
    qi::rule<Iterator,                            Skipper>& end_of_statement;

    // Compounds
    qi::rule<Iterator, FlowControlInstruction(),  Skipper> instr[2];
    qi::rule<Iterator, FlowControlInstruction(),  Skipper> flow_control_instruction;

    // Utility
    qi::rule<Iterator,                            Skipper> not_comma;
    qi::rule<Iterator, bool(),                    Skipper> negation;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct SetEmitParser : qi::grammar<Iterator, SetEmitInstruction(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    SetEmitParser(const ParserContext& context);

    CommonRules<Iterator> common;

    qi::symbols<char, OpCode>& opcodes_setemit;

    // Rule-ified symbols, which can be assigned debug names
    qi::rule<Iterator, OpCode(),                  Skipper> opcode;
    qi::rule<Iterator, unsigned int(),            Skipper> vertex_id;
    qi::rule<Iterator, bool(),                    Skipper> prim_flag;
    qi::rule<Iterator, bool(),                    Skipper> inv_flag;
    qi::rule<Iterator, SetEmitInstruction::Flags(), Skipper> flags;

    // Building blocks
    qi::rule<Iterator,                            Skipper>& end_of_statement;

    // Compounds
    qi::rule<Iterator, SetEmitInstruction(),  Skipper> setemit_instruction;

    // Utility
    qi::rule<Iterator,                            Skipper> not_comma;
    qi::rule<Iterator, bool(),                    Skipper> negation;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct LabelParser : qi::grammar<Iterator, StatementLabel(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    LabelParser(const ParserContext& context);

    CommonRules<Iterator> common;

    qi::rule<Iterator,                            Skipper>& end_of_statement;

    qi::rule<Iterator, std::string(),             Skipper>& identifier;
    qi::rule<Iterator, std::string(),             Skipper>  label;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct DeclarationParser : qi::grammar<Iterator, StatementDeclaration(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    DeclarationParser(const ParserContext& context);

    CommonRules<Iterator> common;

    qi::rule<Iterator, Skipper> string_as;
    qi::rule<Iterator, std::vector<float>(), Skipper> dummy_const;
    qi::rule<Iterator, boost::optional<OutputRegisterInfo::Type>(), Skipper> dummy_semantic;

    qi::symbols<char, OutputRegisterInfo::Type>   output_semantics;

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator, OutputRegisterInfo::Type(),Skipper> output_semantics_rule;

    // Building blocks
    qi::rule<Iterator, std::string(),                 Skipper>& identifier;
    qi::rule<Iterator, InputSwizzlerMask(),           Skipper>& swizzle_mask;
    qi::rule<Iterator, std::vector<float>(),          Skipper> constant;
    qi::rule<Iterator, std::string(),                 Skipper> alias_identifier;
    qi::rule<Iterator, StatementDeclaration::Extra(), Skipper> const_or_semantic;
    qi::rule<Iterator,                                Skipper>& end_of_statement;

    qi::rule<Iterator, StatementDeclaration(),        Skipper> declaration;
    Diagnostics diagnostics;
};

using ParserIterator = SourceTreeIterator;
