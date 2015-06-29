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

#include <vector>
#include <ostream>

#include "nihstro/parser_assembly.h"
#include "nihstro/parser_assembly_private.h"

#include <boost/optional/optional_io.hpp>
#include <boost/spirit/include/qi.hpp>

#define BOOST_TEST_MODULE Parser
#include <boost/test/unit_test.hpp>

// Utility function to parse the given input. Upon match, returns the parsed contents, and returns no value upon failure.
template<typename Attr, typename Parser, typename Skipper>
static boost::optional<Attr> parse(const std::string& input, const Parser& parser, const Skipper& skipper) {
    Attr attr;
    std::string inp(input);
    if (boost::spirit::qi::phrase_parse(inp.begin(), inp.end(), parser, skipper, attr))
        return attr;
    else
        return {};
}

// Utility function to check that the given vectors are equal.
template<typename T>
static void CheckVector(const std::vector<T>& vec, const std::vector<T>& exp) {
    // TODO: Could just check directly for equality, but need an ostream<< operator for that.
    BOOST_CHECK_EQUAL(vec.size(), exp.size());
    for (size_t index = 0; index < vec.size() && index < exp.size(); ++index)
        BOOST_CHECK_EQUAL(vec[index], exp[index]);
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

BOOST_AUTO_TEST_CASE(declaration) {
    ParserContext context;
    DeclarationParser<std::string::iterator> declaration_parser(context);
    AssemblySkipper<std::string::iterator> skipper;

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
    // Constant alias
    auto declaration = parse<StatementDeclaration>(".alias my_alias c5.xy as (1.0, -2.4)", declaration_parser, skipper);
    CheckDeclaration(declaration, { "my_alias", "c5", {}, MakeInputSwizzlerMask<'x', 'y'>(), { { 1.0, -2.4 } } });
    }
}

BOOST_AUTO_TEST_CASE(label) {
    ParserContext context;
    LabelParser<std::string::iterator> label_parser(context);
    AssemblySkipper<std::string::iterator> skipper;

    {
    auto label = parse<StatementLabel>("my_label:", label_parser, skipper);
    BOOST_CHECK(label);
    BOOST_CHECK_EQUAL(*label, "my_label");
    }
}
