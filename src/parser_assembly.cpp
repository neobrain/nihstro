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

#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>

#include "parser_assembly.h"

#include "shader_binary.h"
#include "shader_bytecode.h"

using namespace boost::spirit;
namespace phoenix = boost::phoenix;

class Diagnostics
{
public:
//    Diagnostics();

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
        const utf8_string& tag(info.tag);
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

        // TODO: Specify a source line number
        std::cerr << "Parse error: " << diagnostic << std::endl
                  << std::string(4, ' ') << std::string(begin, newline_iterator) << std::endl
                  << std::string(4 + std::distance(begin, where), ' ') << '^' << std::endl;
    }
};
phoenix::function<ErrorHandler> error_handler;

template<typename Iterator>
struct AssemblySkipper : public qi::grammar<Iterator> {

    AssemblySkipper() : AssemblySkipper::base_type(skip) {

        comments = qi::char_("//") >> *(qi::char_ - qi::eol) >> qi::eol;

        skip = +(comments | ascii::space);
    }

    qi::rule<Iterator> comments;
    qi::rule<Iterator> skip;
};

std::ostream& operator<<(std::ostream& os, const Instruction::OpCode& opcode) {
    // TODO: Should print actual opcode here..
    return os << static_cast<uint32_t>(opcode);
}

template<typename Iterator>
struct CommonRules {
    using Skipper = AssemblySkipper<Iterator>;

    CommonRules(const ParserContext& context) {

        signs.add( "+", +1)
                 ( "-", -1);

        // TODO: Might want to change to only have "x", "y", "z" and "w"
        // TODO: Add rgba/stq masks
        swizzlers.add( "x",    {1, {InputSwizzlerMask::x}} )
                     ( "y",    {1, {InputSwizzlerMask::y}} )
                     ( "z",    {1, {InputSwizzlerMask::z}} )
                     ( "w",    {1, {InputSwizzlerMask::w}} )
                     ( "xy",   {2, {InputSwizzlerMask::x,InputSwizzlerMask::y}} )
                     ( "xz",   {2, {InputSwizzlerMask::x,InputSwizzlerMask::z}} )
                     ( "xw",   {2, {InputSwizzlerMask::x,InputSwizzlerMask::w}} )
                     ( "yz",   {2, {InputSwizzlerMask::y,InputSwizzlerMask::z}} )
                     ( "yw",   {2, {InputSwizzlerMask::y,InputSwizzlerMask::w}} )
                     ( "zw",   {2, {InputSwizzlerMask::z,InputSwizzlerMask::w}} )
                     ( "xyz",  {3, {InputSwizzlerMask::x,InputSwizzlerMask::y,InputSwizzlerMask::z}} )
                     ( "xyw",  {3, {InputSwizzlerMask::x,InputSwizzlerMask::y,InputSwizzlerMask::w}} )
                     ( "xzw",  {3, {InputSwizzlerMask::x,InputSwizzlerMask::z,InputSwizzlerMask::w}} )
                     ( "yzw",  {3, {InputSwizzlerMask::y,InputSwizzlerMask::z,InputSwizzlerMask::w}} )
                     ( "xyzw", {4, {InputSwizzlerMask::x,InputSwizzlerMask::y,InputSwizzlerMask::z,InputSwizzlerMask::w}} );
        swizzle_mask = qi::lexeme[swizzlers];

        // TODO: Something like test5bla should be allowed, too
        identifier = qi::lexeme[+(qi::char_("a-zA-Z_")) >> -+qi::char_("0-9")];
        known_identifier = qi::lexeme[context.identifiers];
        peek_identifier = &identifier;

        uint_after_sign = qi::uint_; // TODO: NOT dot (or alphanum) after this to prevent floats..., TODO: overflows?
        auto sign_with_uint = signs > uint_after_sign;
        index_expression_first_term = (qi::attr(+1) >> qi::uint_) | (peek_identifier > known_identifier);
        index_expression_following_terms = (qi::lit('+') >> peek_identifier > known_identifier) | sign_with_uint;
        index_expression = (-index_expression_first_term)           // the first element has an optional sign
                            >> (*index_expression_following_terms); // following elements have a mandatory sign

        expression = ((-signs) > peek_identifier > known_identifier) >> (-(qi::lit('[') > index_expression > qi::lit(']'))) >> *(qi::lit('.') > swizzle_mask);

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(identifier);
        BOOST_SPIRIT_DEBUG_NODE(uint_after_sign);
        BOOST_SPIRIT_DEBUG_NODE(index_expression);
        BOOST_SPIRIT_DEBUG_NODE(peek_identifier);
        BOOST_SPIRIT_DEBUG_NODE(known_identifier);
        BOOST_SPIRIT_DEBUG_NODE(expression);
        BOOST_SPIRIT_DEBUG_NODE(swizzle_mask);

        diagnostics.Add(swizzle_mask.name(), "Expected swizzle mask after period");
        diagnostics.Add(peek_identifier.name(), "Expected identifier");
        diagnostics.Add(known_identifier.name(), "Unknown identifier");
        diagnostics.Add(uint_after_sign.name(), "Expected integer number after sign");
        diagnostics.Add(index_expression.name(), "Expected index expression between '[' and ']'");
        diagnostics.Add(expression.name(), "Expected expression of a known identifier");
    }

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator, Identifier(),              Skipper> known_identifier;
    qi::rule<Iterator,                            Skipper> peek_identifier;

    // Building blocks
    qi::rule<Iterator, std::string(),             Skipper> identifier;
    qi::rule<Iterator, Expression(),              Skipper> expression;

    qi::symbols<char, int> signs;

    qi::symbols<char, InputSwizzlerMask>          swizzlers;
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

template<typename Iterator>
struct InstructionParser : qi::grammar<Iterator, StatementInstruction(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    InstructionParser(const ParserContext& context)
                : InstructionParser::base_type(instruction),
                  common(context),
                  known_identifier(common.known_identifier),
                  identifier(common.identifier),
                  expression(common.expression),
                  diagnostics(common.diagnostics) {

        // Setup symbol table
        opcodes[0].add
                   ( "ret",   Instruction::OpCode::RET   )
                   ( "flush", Instruction::OpCode::FLUSH );
        opcodes[1].add
                   ( "call",  Instruction::OpCode::CALL  );

        opcodes[2].add
                   ( "mov",   Instruction::OpCode::MOV   )
                   ( "rcp",   Instruction::OpCode::RCP   )
                   ( "rsq",   Instruction::OpCode::RSQ   );
        opcodes[3].add
                   ( "add",   Instruction::OpCode::ADD   )
                   ( "mul",   Instruction::OpCode::MUL   )
                   ( "dp3",   Instruction::OpCode::DP3   )
                   ( "dp4",   Instruction::OpCode::DP4   )
                   ( "max",   Instruction::OpCode::MAX   )
                   ( "min",   Instruction::OpCode::MIN   );
        opcodes[4].add
                   ( "cmp",   Instruction::OpCode::CMP   );

        // Setup rules

        auto comma_rule = qi::lit(',');

        opcode[0] = qi::no_case[qi::lexeme[opcodes[0]]];
        for (int i = 1; i < 5; ++i) {
            // Make sure that a mnemonic is always followed by a space if it expects an argument
            opcode[i] = qi::no_case[qi::lexeme[opcodes[i] >> qi::omit[ascii::blank]]];
        }

        expression_chain[1] = expression;
		for (int i = 2; i < 5; ++i) {
            expression_chain[i] = expression_chain[i - 1] >> comma_rule > expression;
        }

        // e.g. "add o1, t2, t5"
        not_comma = !comma_rule;
        instr[0] = opcode[0] > not_comma;
        instr[1] = opcode[1] > expression_chain[1] > not_comma;
        instr[2] = opcode[2] > expression_chain[2] > not_comma;
        instr[3] = opcode[3] > expression_chain[3] > not_comma;
        instr[4] = opcode[4] > expression_chain[4] > not_comma;

        // TODO: Expect a newline at the end of things...
        instruction %= instr[0] | instr[1] | instr[2] | instr[3] | instr[4];

        // Error handling
        BOOST_SPIRIT_DEBUG_NODE(not_comma);

        BOOST_SPIRIT_DEBUG_NODE(expression_chain[1]);
        BOOST_SPIRIT_DEBUG_NODE(expression_chain[2]);
        BOOST_SPIRIT_DEBUG_NODE(expression_chain[3]);
        BOOST_SPIRIT_DEBUG_NODE(expression_chain[4]);

        BOOST_SPIRIT_DEBUG_NODE(instr[0]);
        BOOST_SPIRIT_DEBUG_NODE(instr[1]);
        BOOST_SPIRIT_DEBUG_NODE(instr[2]);
        BOOST_SPIRIT_DEBUG_NODE(instr[3]);
        BOOST_SPIRIT_DEBUG_NODE(instr[4]);
        BOOST_SPIRIT_DEBUG_NODE(instruction);

        diagnostics.Add(expression_chain[1].name(), "one argument");
        diagnostics.Add(expression_chain[2].name(), "two arguments");
        diagnostics.Add(expression_chain[3].name(), "three arguments");
        diagnostics.Add(expression_chain[4].name(), "four arguments");

        qi::on_error<qi::fail>(instruction, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
    }

    CommonRules<Iterator> common;

    qi::symbols<char, Instruction::OpCode>        opcodes[5]; // indexed by number of arguments

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator, Identifier(),              Skipper>& known_identifier;
    qi::rule<Iterator, Instruction::OpCode(),     Skipper> opcode[5];

    // Building blocks
    qi::rule<Iterator, std::string(),             Skipper>& identifier;
    qi::rule<Iterator, Expression(),              Skipper>& expression;
    qi::rule<Iterator, std::vector<Expression>(), Skipper> expression_chain[5]; // sequence of instruction arguments

    // Compounds
    qi::rule<Iterator, StatementInstruction(),    Skipper> instr[5];
    qi::rule<Iterator, StatementInstruction(),    Skipper> instruction;

    // Utility
    qi::rule<Iterator,                            Skipper> not_comma;

    Diagnostics diagnostics;
};

template<typename Iterator>
struct LabelParser : qi::grammar<Iterator, StatementLabel(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    LabelParser(const ParserContext& context) : LabelParser::base_type(label), common(context), identifier(common.identifier){

        label = identifier >> qi::lit(':');

        BOOST_SPIRIT_DEBUG_NODE(label);
    }

    CommonRules<Iterator> common;

    qi::rule<Iterator, std::string(),             Skipper>& identifier;
    qi::rule<Iterator, std::string(),             Skipper> label;
};

template<typename Iterator>
struct DeclarationParser : qi::grammar<Iterator, StatementDeclaration(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    DeclarationParser(const ParserContext& context)
                : DeclarationParser::base_type(declaration),
                  common(context), known_identifier(common.known_identifier),
                  identifier(common.identifier), diagnostics(common.diagnostics) {

        // Setup symbol table
        output_semantics.add("position", OutputRegisterInfo::POSITION);
        output_semantics.add("color", OutputRegisterInfo::COLOR);
        output_semantics.add("texcoord0", OutputRegisterInfo::TEXCOORD0);
        output_semantics.add("texcoord1", OutputRegisterInfo::TEXCOORD1);
        output_semantics.add("texcoord2", OutputRegisterInfo::TEXCOORD2);
        output_semantics_rule = qi::lexeme[output_semantics];

        // Setup rules

        auto comma_rule = qi::lit(',');

        declaration_output = qi::omit[qi::lexeme["out" >> ascii::blank]] > identifier > context.identifiers > output_semantics;
        declaration_constant = qi::omit[qi::lexeme["const "]] >> identifier > context.identifiers
                               > (qi::repeat(1)[qi::float_]
                                  | (qi::lit('(') > (qi::float_ % qi::lit(',')) > qi::lit(')')));
        declaration_alias = qi::omit[qi::lexeme["alias" >> ascii::blank]] > identifier > context.identifiers;
        declaration = qi::lit('.') > (declaration_output | declaration_constant | declaration_alias);

        // Error handling
        output_semantics_rule.name("output semantic");

        declaration_output.name("output");
        declaration_constant.name("constant");
        declaration_alias.name("alias");

        BOOST_SPIRIT_DEBUG_NODE(declaration);

        qi::on_error<qi::fail>(declaration, error_handler(phoenix::ref(diagnostics), _1, _2, _3, _4));
    }

    CommonRules<Iterator> common;

    qi::symbols<char, OutputRegisterInfo::Type>   output_semantics;

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator, Identifier(),              Skipper>& known_identifier;
    qi::rule<Iterator, OutputRegisterInfo::Type(),Skipper> output_semantics_rule;

    // Building blocks
    qi::rule<Iterator, std::string(),             Skipper>& identifier;

    // Compounds
    qi::rule<Iterator, DeclarationConstant(),     Skipper> declaration_constant;
    qi::rule<Iterator, DeclarationOutput(),       Skipper> declaration_output;
    qi::rule<Iterator, DeclarationAlias(),        Skipper> declaration_alias;

    qi::rule<Iterator, StatementDeclaration(),    Skipper> declaration;
    Diagnostics diagnostics;
};

struct Parser::ParserImpl {
    using Iterator = std::string::iterator;

    ParserImpl(const ParserContext& context) : label(context), instruction(context), declaration(context) {
    }

    void Skip(Iterator& begin, Iterator end) {
        parse(begin, end, skipper);
    }

    bool ParseLabel(Iterator& begin, Iterator end, StatementLabel* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, label, skipper, *content);
    }

    bool ParseInstruction(Iterator& begin, Iterator end, StatementInstruction* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, instruction, skipper, *content);
    }

    bool ParseDeclaration(Iterator& begin, Iterator end, StatementDeclaration* content) {
        assert(content != nullptr);

        return phrase_parse(begin, end, declaration, skipper, *content);
    }

private:
    AssemblySkipper<Iterator>   skipper;

    LabelParser<Iterator>       label;
    InstructionParser<Iterator> instruction;
    DeclarationParser<Iterator> declaration;
};

Parser::Parser(const ParserContext& context) {
    impl = new ParserImpl(context);
};

Parser::~Parser() {
    delete impl;
}

void Parser::Skip(Iterator& begin, Iterator end) {
    impl->Skip(begin, end);
}

bool Parser::ParseLabel(Iterator& begin, Iterator end, StatementLabel* label) {
    return impl->ParseLabel(begin, end, label);
}

bool Parser::ParseInstruction(Iterator& begin, Iterator end, StatementInstruction* instruction) {
    return impl->ParseInstruction(begin, end, instruction);
}

bool Parser::ParseDeclaration(Iterator& begin, Iterator end, StatementDeclaration* declaration) {
    return impl->ParseDeclaration(begin, end, declaration);
}
