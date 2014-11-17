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

        // First number need not have a sign, the others do
        integer_with_sign = signs > qi::uint_;
        optionally_signed_int = qi::attr(+1) > qi::uint_;
        array_indices = qi::lit('[') > (optionally_signed_int | integer_with_sign) > *integer_with_sign > qi::lit(']');

        expression = (-signs) >> known_identifier >> (-array_indices) >> *(qi::lit('.') > swizzle_mask);

        // Error handling
        expression.name("expression");
        identifier.name("identifier");
        known_identifier.name("known identifier");
        swizzle_mask.name("swizzle mask");
    }

    // Rule-ified symbols, which can be assigned names
    qi::rule<Iterator, Identifier(),              Skipper> known_identifier;

    // Building blocks
    qi::rule<Iterator, std::string(),             Skipper> identifier;
    qi::rule<Iterator, Expression(),              Skipper> expression;

    qi::symbols<char, int> signs;

    qi::symbols<char, InputSwizzlerMask>          swizzlers;
    qi::rule<Iterator, InputSwizzlerMask(),       Skipper> swizzle_mask;

private:
    qi::rule<Iterator, IntegerWithSign(),         Skipper> integer_with_sign;
    qi::rule<Iterator, IntegerWithSign(),         Skipper> optionally_signed_int;
    qi::rule<Iterator, IndexExpression(),         Skipper> array_indices;
};

template<typename Iterator>
struct InstructionParser : qi::grammar<Iterator, StatementInstruction(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    InstructionParser(const ParserContext& context) : InstructionParser::base_type(start), common(context), identifier(common.identifier), known_identifier(common.known_identifier), expression(common.expression) {

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
        start %= instr[0] | instr[1] | instr[2] | instr[3] | instr[4];

        // Error handling
        not_comma.name("not comma");

        expression_chain[1].name("1 argument");
        expression_chain[2].name("2 arguments");
        expression_chain[3].name("3 arguments");
        expression_chain[4].name("4 arguments");

        instr[0].name("instr[0]");
        instr[1].name("instr[1]");
        instr[2].name("instr[2]");
        instr[3].name("instr[3]");
        instr[4].name("instr[4]");
        start.name("instruction statement");

        debug(instr[0]);
        debug(instr[1]);
        debug(instr[2]);
        debug(instr[3]);
        debug(instr[4]);
        debug(start);

        // TODO: Make these error messages more helpful...
        // _1: Iterator first
        // _2: Iterator last
        // _3: Iterator err_pos
        // _4: spirit::info const &what
		qi::on_error<qi::fail>
		(
			start
          , std::cout
                << phoenix::val("Error! Expected ")
                << _4                               // what failed?
                << phoenix::val(" here: \"")
                << phoenix::construct<std::string>(_1, _3) + "___" + phoenix::construct<std::string>(_3+1, _2)
                << phoenix::val("\"")
                << std::endl
		);
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
    qi::rule<Iterator, StatementInstruction(),    Skipper> start;

    // Utility
    qi::rule<Iterator,                            Skipper> not_comma;
};

template<typename Iterator>
struct LabelParser : qi::grammar<Iterator, StatementLabel(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    LabelParser(const ParserContext& context) : LabelParser::base_type(label), common(context), identifier(common.identifier){

        label = identifier >> qi::lit(':');

        label.name("label");

        debug(label);
    }

    CommonRules<Iterator> common;

    qi::rule<Iterator, std::string(),             Skipper>& identifier;
    qi::rule<Iterator, std::string(),             Skipper> label;
};

template<typename Iterator>
struct DeclarationParser : qi::grammar<Iterator, StatementDeclaration(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    DeclarationParser(const ParserContext& context) : DeclarationParser::base_type(declaration), common(context), identifier(common.identifier), known_identifier(common.known_identifier) {

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
        declaration.name("declaration");

        debug(declaration);

        // TODO: Make these error messages more helpful...
        // _1: Iterator first
        // _2: Iterator last
        // _3: Iterator err_pos
        // _4: spirit::info const &what
		qi::on_error<qi::fail>
		(
			declaration
          , std::cout
                << phoenix::val("Error! Expected ")
                << _4                               // what failed?
                << phoenix::val(" here: \"")
                << phoenix::construct<std::string>(_1, _3) + "___" + phoenix::construct<std::string>(_3+1, _2)
                << phoenix::val("\"")
                << std::endl
		);
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
