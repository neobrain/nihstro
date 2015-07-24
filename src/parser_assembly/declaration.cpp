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


#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi.hpp>

#include "nihstro/parser_assembly.h"
#include "nihstro/parser_assembly_private.h"

#include "nihstro/shader_binary.h"
#include "nihstro/shader_bytecode.h"

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

// Manually define a swap() overload for qi::hold to work.
/*namespace boost {
namespace spirit {
void swap(nihstro::Condition& a, nihstro::Condition& b) {
    boost::fusion::swap(a, b);
}
}
}*/

template<>
DeclarationParser<ParserIterator>::DeclarationParser(const ParserContext& context)
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

        dummy_const = qi::attr(std::vector<float>());
        dummy_semantic = qi::attr(boost::optional<OutputRegisterInfo::Type>());

        // match a constant or a semantic, and fill the respective other one with a dummy
        const_or_semantic = (dummy_const >> output_semantics_rule) | (constant >> dummy_semantic);

        // TODO: Would like to use +ascii::blank instead, but somehow that fails to parse lines like ".alias name o2.xy texcoord0" correctly
        string_as = qi::omit[qi::no_skip[*/*+*/ascii::blank >> qi::lit("as") >> +ascii::blank]];

        declaration = ((qi::lit('.') > alias_identifier) >> identifier >> -(qi::lit('-') > identifier) >> -(qi::lit('.') > swizzle_mask))
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
