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
    IntegerWithSign,
    (int, sign)
    (unsigned, value)
)

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

template<>
CommonRules<ParserIterator>::CommonRules(const ParserContext& context) {
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
               ( "lit",      OpCode::Id::LIT      )
               ( "flr",      OpCode::Id::FLR      )
               ( "rcp",      OpCode::Id::RCP      )
               ( "rsq",      OpCode::Id::RSQ      )
               ( "mov",      OpCode::Id::MOV      );
    opcodes_float[2].add
               ( "add",      OpCode::Id::ADD      )
               ( "dp3",      OpCode::Id::DP3      )
               ( "dp4",      OpCode::Id::DP4      )
               ( "dph",      OpCode::Id::DPH      )
               ( "dst",      OpCode::Id::DST      )
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
               ( "break",    OpCode::Id::BREAK    )
               ( "breakc",   OpCode::Id::BREAKC   )
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
        sign_with_uint = signs > uint_after_sign;
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
