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

#include "nihstro/parser_assembly.h"
#include "nihstro/parser_assembly_private.h"

#include <boost/optional/optional_io.hpp>
#include <boost/spirit/include/qi.hpp>

#define BOOST_TEST_MODULE Parser
#include <boost/test/unit_test.hpp>

// Implement some ostream<< operators for BOOST_CHECK*
namespace std {

template<typename T>
std::ostream& operator << (std::ostream& os, const std::vector<T>& vec) {
    auto it = vec.begin();
    os << "{";
    if (!vec.empty())
        os << " " << *it;
    while (it != vec.end() && ++it != vec.end()) {
        os << ", " << *it;
    }
    os << " }";
    return os;
}

std::ostream& operator << (std::ostream& os, const nihstro::Expression& expr) {
    if (expr.signed_identifier.sign)
        os << *expr.signed_identifier.sign;
    os << expr.signed_identifier.identifier;
    if (expr.index)
        os << "[" << *expr.index << "]";
    for (auto& mask : expr.swizzle_masks)
        os << "." << mask;
    return os;
}

std::ostream& operator << (std::ostream& os, const nihstro::IndexExpression& expr) {
    for (size_t i = 0; i < expr.GetCount(); ++i) {
        if (i != 0)
            os << ", ";

        if (expr.IsRawIndex(i)) {
            os << expr.GetRawIndex(i);
        } else if (expr.IsAddressRegisterIdentifier(i)) {
            os << expr.GetAddressRegisterIdentifier(i);
        } else {
            os << "?";
        }
    }
    return os;
}

std::ostream& operator << (std::ostream& os, const nihstro::IntegerWithSign& num) {
    os << num.GetValue();
    return os;
}

std::ostream& operator << (std::ostream& os, const nihstro::Instruction::FlowControlType::Op& op) {
    if (op == Instruction::FlowControlType::And)
        os << "&&";
    else if (op == Instruction::FlowControlType::Or)
        os << "||";
    else
        os << "??";

    return os;
}

std::ostream& operator << (std::ostream& os, const nihstro::ConditionInput& inp) {
    if (inp.invert)
        os << "!";

    os << inp.identifier;
    if (inp.swizzler_mask)
        os << "." << *inp.swizzler_mask;

    return os;
}

std::ostream& operator << (std::ostream& os, const nihstro::Condition& cond) {
    os << "{ " << cond.input1;

    if (cond.op != Instruction::FlowControlType::JustX)
        os << " " << cond.op << " " << cond.input2;

    os << " }";
    return os;
}

} // namespace std

// Utility comparison operators
namespace nihstro {
bool operator == (const IntegerWithSign& a, const IntegerWithSign& b) {
    return a.sign == b.sign && a.value == b.value;
}

bool operator == (const Expression& a, const Expression& b) {
    bool ret = true;
    ret &= a.signed_identifier.sign == b.signed_identifier.sign;
    ret &= a.signed_identifier.identifier == b.signed_identifier.identifier;
    ret &= a.index == b.index;
    ret &= a.swizzle_masks == b.swizzle_masks;
    return ret;
}

bool operator == (const ConditionInput& a, const ConditionInput& b) {
    return a.invert == b.invert && a.identifier == b.identifier && a.swizzler_mask == b.swizzler_mask;
}

bool operator == (const Condition& a, const Condition& b) {
    return a.input1 == b.input1 && a.input2 == b.input2 && a.op == b.op;
}

} // namespace nihstro

// Utility function to parse the given input. Upon match, returns the parsed contents, and returns no value upon failure.
template<typename Attr, typename Parser, typename Skipper>
static boost::optional<Attr> parse(const std::string& input, const Parser& parser, const Skipper& skipper) {
    BOOST_TEST_MESSAGE(("Parsing \"" + input + "\"").c_str());
    Attr attr;
    SourceTree inp;
    inp.code = input;
    if (boost::spirit::qi::phrase_parse(inp.begin(), inp.end(), parser, skipper, attr))
        return attr;
    else
        return {};
}

// Utility function to check that the given vectors are equal.
template<typename T>
static void CheckVector(const std::vector<T>& vec, const std::vector<T>& exp) {
    // TODO: Could just check directly for equality, but need an ostream<< operator for that.
    BOOST_CHECK_EQUAL(vec, exp);
}

// Utility function to convert a compile-time character to the corresponding swizzle mask component
template<char a>
static InputSwizzlerMask::Component MakeSwizzlerMaskComponent() {
    static_assert(a == 'x' || a == 'y' || a == 'z' || a == 'w', "Invalid component");
    if (a == 'x') return InputSwizzlerMask::x;
    else if (a == 'y') return InputSwizzlerMask::y;
    else if (a == 'z') return InputSwizzlerMask::z;
    else return InputSwizzlerMask::w;
}

// Utility function to convert a series of up to four characters to the corresponding swizzle mask
template<char a>
static InputSwizzlerMask MakeInputSwizzlerMask() {
    return { 1, { MakeSwizzlerMaskComponent<a>() } };
}
template<char a, char b>
static InputSwizzlerMask MakeInputSwizzlerMask() {
    return { 2, { MakeSwizzlerMaskComponent<a>(), MakeSwizzlerMaskComponent<b>() } };
}
template<char a, char b, char c>
static InputSwizzlerMask MakeInputSwizzlerMask() {
    return { 3, { MakeSwizzlerMaskComponent<a>(), MakeSwizzlerMaskComponent<b>(), MakeSwizzlerMaskComponent<c>() } };
}
template<char a, char b, char c, char d>
static InputSwizzlerMask MakeInputSwizzlerMask() {
    return { 4, { MakeSwizzlerMaskComponent<a>(), MakeSwizzlerMaskComponent<b>(), MakeSwizzlerMaskComponent<c>(), MakeSwizzlerMaskComponent<d>() } };
}

// Utility function to check that parsing of the first declaration statement was successfull and that the result matches the second declaration statements.
static void CheckDeclaration(const boost::optional<StatementDeclaration>& declaration, const StatementDeclaration& exp) {
    if (!declaration) {
        BOOST_CHECK(false);
    } else {
        BOOST_CHECK_EQUAL(declaration->alias_name, exp.alias_name);
        BOOST_CHECK_EQUAL(declaration->identifier_start, exp.identifier_start);
        BOOST_CHECK_EQUAL(declaration->identifier_end, exp.identifier_end);
        BOOST_CHECK_EQUAL(declaration->swizzle_mask, exp.swizzle_mask);
        CheckVector(declaration->extra.constant_value, exp.extra.constant_value);
        BOOST_CHECK_EQUAL(declaration->extra.output_semantic, exp.extra.output_semantic);
    }
}

BOOST_AUTO_TEST_CASE(declaration) {
    ParserContext context;
    DeclarationParser<ParserIterator> declaration_parser(context);
    AssemblySkipper<ParserIterator> skipper;

    {
    // Plain alias
    auto declaration = parse<StatementDeclaration>(".alias my_alias r5.xy", declaration_parser, skipper);
    CheckDeclaration(declaration, { "my_alias", "r5", {}, MakeInputSwizzlerMask<'x', 'y'>() });
    }

    {
    // Array alias
    auto declaration = parse<StatementDeclaration>(".alias my_alias r5-r10", declaration_parser, skipper);
    CheckDeclaration(declaration, { "my_alias", "r5", std::string("r10") });
    }

    {
    // Output alias
    auto declaration = parse<StatementDeclaration>(".alias my_alias o5.xyz as texcoord0", declaration_parser, skipper);
    CheckDeclaration(declaration, { "my_alias", "o5", {}, MakeInputSwizzlerMask<'x', 'y', 'z'>(), { {}, OutputRegisterInfo::TEXCOORD0 } });
    }

    {
    // Output alias without output semantic
    BOOST_CHECK_THROW(parse<StatementDeclaration>(".alias my_alias o5.xyz", declaration_parser, skipper),
                      std::runtime_error);
    }

    {
    // Constant alias
    auto declaration = parse<StatementDeclaration>(".alias my_alias c5.xy as (1.0, -2.4)", declaration_parser, skipper);
    CheckDeclaration(declaration, { "my_alias", "c5", {}, MakeInputSwizzlerMask<'x', 'y'>(), { { 1.0, -2.4 } } });
    }
}

BOOST_AUTO_TEST_CASE(label) {
    ParserContext context;
    LabelParser<ParserIterator> label_parser(context);
    AssemblySkipper<ParserIterator> skipper;

    {
    auto label = parse<StatementLabel>("my_label:", label_parser, skipper);
    BOOST_CHECK(label);
    BOOST_CHECK_EQUAL(*label, "my_label");
    }
}

static void CheckParsedArithmeticInstruction(const boost::optional<FloatOpInstruction>& result, const FloatOpInstruction& exp) {
    if (!result) {
        BOOST_CHECK(false);
    } else {
        BOOST_CHECK_EQUAL(result->opcode, exp.opcode);
        CheckVector(result->expressions, exp.expressions);
    }
}

BOOST_AUTO_TEST_CASE(arithmetic) {
    ParserContext context;

    FloatOpParser<ParserIterator> arithmetic_parser(context);
    AssemblySkipper<ParserIterator> skipper;

    {
    // one-argument instruction
    auto result = parse<FloatOpInstruction>("mova r2.xy", arithmetic_parser, skipper);
    Expression r2_xy = { { {}, "r2" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    CheckParsedArithmeticInstruction(result, { nihstro::OpCode::Id::MOVA, { r2_xy } });
    }

    {
    // two-argument instruction
    auto result = parse<FloatOpInstruction>("mov o0.wz, r2.xy", arithmetic_parser, skipper);
    Expression o0_wz = { { {}, "o0" }, {}, { MakeInputSwizzlerMask<'w', 'z'>() } };
    Expression r2_xy = { { {}, "r2" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    CheckParsedArithmeticInstruction(result, { nihstro::OpCode::Id::MOV, { o0_wz, r2_xy } });
    }

    {
    // two-argument instruction with trivial register index
    auto result = parse<FloatOpInstruction>("mov o0.wz, r2[5].xy", arithmetic_parser, skipper);
    IndexExpression index_expr;
    index_expr.emplace_back(IntegerWithSign{ +1, 5 });
    Expression o0_wz = { { {}, "o0" }, {}, { MakeInputSwizzlerMask<'w', 'z'>() } };
    Expression r2_xy = { { {}, "r2" }, index_expr, { MakeInputSwizzlerMask<'x', 'y'>() } };
    CheckParsedArithmeticInstruction(result, { nihstro::OpCode::Id::MOV, { o0_wz, r2_xy } });
    }

    {
    // two-argument instruction with nontrivial register index
    auto result = parse<FloatOpInstruction>("mov o0.wz, r2[5+a1-4].xy", arithmetic_parser, skipper);
    IndexExpression index_expr;
    index_expr.emplace_back(IntegerWithSign{ +1, 5 });
    index_expr.emplace_back("a1");
    index_expr.emplace_back(IntegerWithSign{ -1, 4 });
    Expression o0_wz = { { {}, "o0" }, {}, { MakeInputSwizzlerMask<'w', 'z'>() } };
    Expression r2_xy = { { {}, "r2" }, index_expr, { MakeInputSwizzlerMask<'x', 'y'>() } };
    CheckParsedArithmeticInstruction(result, { nihstro::OpCode::Id::MOV, { o0_wz, r2_xy } });
    }

    {
    // three-argument instruction
    auto result = parse<FloatOpInstruction>("add o0.xy, r2.xy, r3.xy", arithmetic_parser, skipper);
    Expression o0_xy = { { {}, "o0" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    Expression r2_xy = { { {}, "r2" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    Expression r3_xy = { { {}, "r3" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    CheckParsedArithmeticInstruction(result, { nihstro::OpCode::Id::ADD, { o0_xy, r2_xy, r3_xy } });
    }

    {
    // four-argument instruction
    auto result = parse<FloatOpInstruction>("mad o0.xy, r2.xy, r3.xy, v4.xy", arithmetic_parser, skipper);
    Expression o0_xy = { { {}, "o0" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    Expression r2_xy = { { {}, "r2" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    Expression r3_xy = { { {}, "r3" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    Expression v4_xy = { { {}, "v4" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    CheckParsedArithmeticInstruction(result, { nihstro::OpCode::Id::MAD, { o0_xy, r2_xy, r3_xy, v4_xy } });
    }
}

static void CheckParsedFlowControlInstruction(const boost::optional<FlowControlInstruction>& result, const FlowControlInstruction& exp) {
    if (!result) {
        BOOST_CHECK(false);
    } else {
        BOOST_CHECK_EQUAL(result->opcode, exp.opcode);
        BOOST_CHECK_EQUAL(result->target_label, exp.target_label);
        BOOST_CHECK_EQUAL(result->return_label, exp.return_label);
        BOOST_CHECK_EQUAL(result->condition, exp.condition);
    }
}

BOOST_AUTO_TEST_CASE(flowcontrol) {
    ParserContext context;
    FlowControlParser<ParserIterator> parser(context);
    AssemblySkipper<ParserIterator> skipper;

    {
    // If-conditionals with two arguments
    auto result = parse<FlowControlInstruction>("if cc.x && !cc.y", parser, skipper);
    Condition cond = { { false, "cc", MakeInputSwizzlerMask<'x'>() }, Instruction::FlowControlType::And, { true, "cc", MakeInputSwizzlerMask<'y'>() } };
    CheckParsedFlowControlInstruction(result, { nihstro::OpCode::Id::GEN_IF, "__dummy", {}, cond } );
    }

    {
    // If-conditionals with one argument with one component
    auto result = parse<FlowControlInstruction>("if cc.x", parser, skipper);
    Condition cond = { { false, "cc", MakeInputSwizzlerMask<'x'>() }, Instruction::FlowControlType::JustX };
    CheckParsedFlowControlInstruction(result, { nihstro::OpCode::Id::GEN_IF, "__dummy", {}, cond } );
    }

    {
    // If-conditionals with one argument with two components
    auto result = parse<FlowControlInstruction>("if !cc.xy", parser, skipper);
    Condition cond = { { true, "cc", MakeInputSwizzlerMask<'x', 'y'>() }, Instruction::FlowControlType::JustX };
    CheckParsedFlowControlInstruction(result, { nihstro::OpCode::Id::GEN_IF, "__dummy", {}, cond });
    }

    {
    // Loop instruction
    auto result = parse<FlowControlInstruction>("loop i3.y", parser, skipper);
    Condition cond = { { false, "i3", MakeInputSwizzlerMask<'y'>() }, Instruction::FlowControlType::JustX };
    CheckParsedFlowControlInstruction(result, { OpCode::Id::LOOP, "__dummy", {}, cond });
    }
}

static void CheckParsedCompareInstruction(const boost::optional<CompareInstruction>& result, const CompareInstruction& exp) {
    if (!result) {
        BOOST_CHECK(false);
    } else {
        BOOST_CHECK_EQUAL(result->opcode, exp.opcode);
        CheckVector(result->arguments, exp.arguments);
        CheckVector(result->ops, exp.ops);
    }
}

BOOST_AUTO_TEST_CASE(compare) {
    ParserContext context;
    CompareParser<ParserIterator> parser(context);
    AssemblySkipper<ParserIterator> skipper;

    {
    // Two separate comparisons
    auto result = parse<CompareInstruction>("cmp r5.xy, r2.zx, ==, <=", parser, skipper);
    Expression r5_xy = { { {}, "r5" }, {}, { MakeInputSwizzlerMask<'x', 'y'>() } };
    Expression r2_zx = { { {}, "r2" }, {}, { MakeInputSwizzlerMask<'z', 'x'>() } };
    CheckParsedCompareInstruction(result, { OpCode::Id::CMP, { r5_xy, r2_zx }, { Instruction::Common::CompareOpType::Equal, Instruction::Common::CompareOpType::LessEqual } });
    }
}
