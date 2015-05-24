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

#include <boost/spirit/include/qi.hpp>

#include "nihstro/parser_assembly.h"

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
    IntegerWithSign,
    (int, sign)
    (unsigned, value)
)

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
    StatementInstruction,
    (OpCode, opcode)
    (std::vector<Expression>, expressions)
)

BOOST_FUSION_ADAPT_STRUCT(
    CompareInstruction,
    (OpCode, opcode)
    (std::vector<Expression>, arguments)
    (std::vector<Instruction::Common::CompareOpType::Op>, ops)
)

BOOST_FUSION_ADAPT_STRUCT(
    FlowControlInstruction,
    (OpCode, opcode)
    (std::string, target_label)
    (boost::optional<std::string>, return_label)
    (boost::optional<Condition>, condition)
)

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

BOOST_FUSION_ADAPT_STRUCT(
    StatementDeclaration::Extra,
    (std::vector<float>, constant_value)
    (boost::optional<OutputRegisterInfo::Type>, output_semantic)
)

BOOST_FUSION_ADAPT_STRUCT(
    StatementDeclaration,
    (std::string, alias_name)
    (Identifier, identifier_start)
    (boost::optional<Identifier>, identifier_end)
    (boost::optional<InputSwizzlerMask>, swizzle_mask)
    (StatementDeclaration::Extra, extra)
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
phoenix::function<ErrorHandler> error_handler;

template<typename Iterator>
struct AssemblySkipper : public qi::grammar<Iterator> {

    AssemblySkipper() : AssemblySkipper::base_type(skip) {
        comments = qi::char_("//") >> *(qi::char_ - qi::eol);

        skip = +(comments | ascii::blank);
    }

    qi::rule<Iterator> comments;
    qi::rule<Iterator> skip;
};

namespace std {

std::ostream& operator<<(std::ostream& os, const OpCode& opcode) {
    // TODO: Should print actual opcode here..
    return os << static_cast<uint32_t>(static_cast<OpCode::Id>(opcode));
}

}

/**
 * Implementation of transform_attribute from std::vector<InputSwizzlerMask::Component> to InputSwizzlerMask.
 * This eases swizzle mask parsing a lot.
 */
namespace boost { namespace spirit { namespace traits {
template<>
struct transform_attribute<InputSwizzlerMask, std::vector<InputSwizzlerMask::Component>, qi::domain>
{
    using Exposed = InputSwizzlerMask;

    using type = std::vector<InputSwizzlerMask::Component>;

    static void post(Exposed& val, const type& attr) {
        val.num_components = attr.size();
        for (size_t i = 0; i < attr.size(); ++i)
            val.components[i] = attr[i];
    }

    static type pre(Exposed& val) {
        type vec;
        for (int i = 0; i < val.num_components; ++i)
            vec.push_back(val.components[i]);
        return vec;
    }

    static void fail(Exposed&) { }
};
}}} // namespaces

template<typename Iterator>
struct CommonRules {
    using Skipper = AssemblySkipper<Iterator>;

    CommonRules(const ParserContext& context) {

        // Setup symbol table
        opcodes_trivial.add
                   ( "nop",      OpCode::Id::NOP      )
                   ( "end",      OpCode::Id::END      )
                   ( "emit",     OpCode::Id::EMIT     )
                   ( "else",     OpCode::Id::ELSE     )
                   ( "endif",    OpCode::Id::ENDIF    )
                   ( "endloop",  OpCode::Id::ENDLOOP  );

        opcodes_float[0].add
                   ( "mova",     OpCode::Id::MOVA     );

        opcodes_float[1].add
                   ( "exp",      OpCode::Id::EX2      )
                   ( "log",      OpCode::Id::LG2      )
                   ( "flr",      OpCode::Id::FLR      )
                   ( "rcp",      OpCode::Id::RCP      )
                   ( "rsq",      OpCode::Id::RSQ      )
                   ( "mov",      OpCode::Id::MOV      );
        opcodes_float[2].add
                   ( "add",      OpCode::Id::ADD      )
                   ( "dp3",      OpCode::Id::DP3      )
                   ( "dp4",      OpCode::Id::DP4      )
                   ( "dph",      OpCode::Id::DPH      )
                   ( "mul",      OpCode::Id::MUL      )
                   ( "sge",      OpCode::Id::SGE      )
                   ( "slt",      OpCode::Id::SLT      )
                   ( "max",      OpCode::Id::MAX      )
                   ( "min",      OpCode::Id::MIN      );
        opcodes_float[3].add
                   ( "mad",      OpCode::Id::MAD      );

        opcodes_compare.add
                   ( "cmp",      OpCode::Id::CMP      );

        opcodes_flowcontrol[0].add
                   ( "break",    OpCode::Id::BREAKC   )
                   ( "if",       OpCode::Id::GEN_IF   )
                   ( "loop",     OpCode::Id::LOOP     );
        opcodes_flowcontrol[1].add
                   ( "jmp",      OpCode::Id::GEN_JMP  )
                   ( "call",     OpCode::Id::GEN_CALL );

        opcodes_setemit.add
                   ( "setemitraw", OpCode::Id::SETEMIT );

        signs.add( "+", +1)
                 ( "-", -1);

        // TODO: Add rgba/stq masks
        swizzlers.add
                     ( "x",    InputSwizzlerMask::x )
                     ( "y",    InputSwizzlerMask::y )
                     ( "z",    InputSwizzlerMask::z )
                     ( "w",    InputSwizzlerMask::w );

        // TODO: Make sure this is followed by a space or *some* separator
        // TODO: Use qi::repeat(1,4)(swizzlers) instead of Kleene [failed to work when I tried, so make this work!]
        // TODO: Use qi::lexeme[swizzlers] [crashed when I tried, so make this work!]
        swizzle_mask = qi::attr_cast<InputSwizzlerMask, std::vector<InputSwizzlerMask::Component>>(*swizzlers);

        identifier = qi::lexeme[qi::char_("a-zA-Z_") >> *qi::char_("a-zA-Z0-9_")];
        peek_identifier = &identifier;

        uint_after_sign = qi::uint_; // TODO: NOT dot (or alphanum) after this to prevent floats..., TODO: overflows?
        auto sign_with_uint = signs > uint_after_sign;
        index_expression_first_term = (qi::attr(+1) >> qi::uint_) | (peek_identifier > identifier);
        index_expression_following_terms = (qi::lit('+') >> peek_identifier > identifier) | sign_with_uint;
        index_expression = (-index_expression_first_term)           // the first element has an optional sign
                            >> (*index_expression_following_terms); // following elements have a mandatory sign

        expression = ((-signs) > peek_identifier > identifier) >> (-(qi::lit('[') > index_expression > qi::lit(']'))) >> *(qi::lit('.') > swizzle_mask);

        end_of_statement = qi::omit[qi::eol | qi::eoi];

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(identifier);
        BOOST_SPIRIT_DEBUG_NODE(uint_after_sign);
        BOOST_SPIRIT_DEBUG_NODE(index_expression);
        BOOST_SPIRIT_DEBUG_NODE(peek_identifier);
        BOOST_SPIRIT_DEBUG_NODE(expression);
        BOOST_SPIRIT_DEBUG_NODE(swizzle_mask);
        BOOST_SPIRIT_DEBUG_NODE(end_of_statement);

        diagnostics.Add(swizzle_mask.name(), "Expected swizzle mask after period");
        diagnostics.Add(peek_identifier.name(), "Expected identifier");
        diagnostics.Add(uint_after_sign.name(), "Expected integer number after sign");
        diagnostics.Add(index_expression.name(), "Expected index expression between '[' and ']'");
        diagnostics.Add(expression.name(), "Expected expression of a known identifier");
        diagnostics.Add(end_of_statement.name(), "Expected end of statement");
    }

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
    qi::rule<Iterator, unsigned int(),            Skipper> uint_after_sign;
};

template<typename Iterator, bool require_end_of_line>
struct TrivialOpParser : qi::grammar<Iterator, OpCode(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    TrivialOpParser(const ParserContext& context)
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

    FloatOpParser(const ParserContext& context)
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

    CompareParser(const ParserContext& context)
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

    FlowControlParser(const ParserContext& context)
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

    SetEmitParser(const ParserContext& context)
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

    LabelParser(const ParserContext& context)
                : LabelParser::base_type(label), common(context),
                  end_of_statement(common.end_of_statement),
                  identifier(common.identifier),
                  diagnostics(common.diagnostics) {

        label = identifier >> qi::lit(':') > end_of_statement;

        BOOST_SPIRIT_DEBUG_NODE(label);

        qi::on_error<qi::fail>(label, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
    }

    CommonRules<Iterator> common;

    qi::rule<Iterator,                            Skipper>& end_of_statement;

    qi::rule<Iterator, std::string(),             Skipper>& identifier;
    qi::rule<Iterator, std::string(),             Skipper>  label;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct DeclarationParser : qi::grammar<Iterator, StatementDeclaration(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    DeclarationParser(const ParserContext& context)
                : DeclarationParser::base_type(declaration),
                  common(context),
                  identifier(common.identifier), swizzle_mask(common.swizzle_mask),
                  end_of_statement(common.end_of_statement),
                  diagnostics(common.diagnostics) {

        // Setup symbol table
        output_semantics.add("position", OutputRegisterInfo::POSITION);
        output_semantics.add("quaternion", OutputRegisterInfo::QUATERNION);
        output_semantics.add("color", OutputRegisterInfo::COLOR);
        output_semantics.add("texcoord0", OutputRegisterInfo::TEXCOORD0);
        output_semantics.add("texcoord1", OutputRegisterInfo::TEXCOORD1);
        output_semantics.add("texcoord2", OutputRegisterInfo::TEXCOORD2);
        output_semantics.add("view", OutputRegisterInfo::VIEW);
        output_semantics_rule = qi::lexeme[output_semantics];

        // Setup rules

        alias_identifier = qi::omit[qi::lexeme["alias" >> ascii::blank]] > identifier;

        // e.g. 5.4 or (1.1, 2, 3)
        constant = (qi::repeat(1)[qi::float_]
                                  | (qi::lit('(') > (qi::float_ % qi::lit(',')) > qi::lit(')')));

        auto dummy_const = qi::attr(std::vector<float>());
        auto dummy_semantic = qi::attr(boost::optional<OutputRegisterInfo::Type>());

        // match a constant or a semantic, and fill the respective other one with a dummy
        const_or_semantic = (dummy_const >> output_semantics_rule) | (constant >> dummy_semantic);

        auto declaration_begin = ((qi::lit('.') > alias_identifier) >> identifier >> -(qi::lit('-') > identifier) >> -(qi::lit('.') > swizzle_mask));

        // TODO: Would like to use +ascii::blank instead, but somehow that fails to parse lines like ".alias name o2.xy texcoord0" correctly
        auto string_as = qi::omit[qi::no_skip[*/*+*/ascii::blank >> qi::lit("as") >> +ascii::blank]];

        declaration = declaration_begin
                       >> (
                            (string_as > const_or_semantic)
                            | (dummy_const >> dummy_semantic)
                          )
                       > end_of_statement;

        // Error handling
        output_semantics_rule.name("output semantic after \"as\"");
        alias_identifier.name("known preprocessor directive (i.e. alias).");
        const_or_semantic.name("constant or semantic after \"as\"");

        BOOST_SPIRIT_DEBUG_NODE(output_semantics_rule);
        BOOST_SPIRIT_DEBUG_NODE(constant);
        BOOST_SPIRIT_DEBUG_NODE(alias_identifier);
        BOOST_SPIRIT_DEBUG_NODE(const_or_semantic);
        BOOST_SPIRIT_DEBUG_NODE(declaration);

        qi::on_error<qi::fail>(declaration, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
    }

    CommonRules<Iterator> common;

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

struct Parser::ParserImpl {
    using Iterator = std::string::iterator;

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
