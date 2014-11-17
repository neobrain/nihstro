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

#include <array>
#include <vector>
#include <ostream>
#include <cstdint>

#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/at_c.hpp>

#include <boost/spirit/include/qi_symbols.hpp>

#include "shader_binary.h"
#include "shader_bytecode.h"

struct InputSwizzlerMask {
    int num_components;

    enum Component : uint8_t {
        x = 0,
        y = 1,
        z = 2,
        w = 3,
    };
    std::array<Component,4> components;

    static InputSwizzlerMask FullMask() {
        return { 4, {x,y,z,w} };
    }

    bool operator == (const InputSwizzlerMask& oth) const {
        return this->num_components == oth.num_components && this->components == oth.components;
    }

	// TODO: Move to implementation?
    friend std::ostream& operator<<(std::ostream& os, const Component& v) {
        switch(v) {
            case x:  return os << "x";
            case y:  return os << "y";
            case z:  return os << "z";
            case w:  return os << "w";
            default: return os << "?";
        }
    }
    friend std::ostream& operator<<(std::ostream& os, const InputSwizzlerMask& v) {
        if (!v.num_components)
            return os << "(empty_mask)";

        for (int i = 0; i < v.num_components; ++i)
            os << v.components[i];

        return os;
    }
};

// Identifer index in identifier list
using Identifier = int;

// A sign, i.e. +1 or -1
using Sign = int;

struct IntegerWithSign : boost::fusion::vector<int, unsigned int> {
    int GetValue() const {
        return boost::fusion::at_c<0>(*this) * boost::fusion::at_c<1>(*this);
    }
};

// Raw index + address register index
struct IndexExpression : std::vector</*boost::variant<*/IntegerWithSign/*, Identifier>*/> {
//    ArrayIndex() = default;
    int GetCount() const {
        return this->size();
    }

    bool IsRawIndex(int arg) const {
        return true/*(*this)[arg].which() == 0*/;
    }

    int GetRawIndex(int arg) const {
        return /*boost::get<int>(*/(*this)[arg]/*)*/.GetValue();
    }

/*    bool IsAddressRegisterIndex(int arg) const {
        return (*this)[arg].which() == 1;
    }

    Identifier GetAddressRegisterIndex(int arg) const {
        return boost::get<Identifier>((*this)[arg]);
    }*/
};

struct Expression : boost::fusion::vector<boost::fusion::vector<boost::optional<Sign>, Identifier>, boost::optional<IndexExpression>, std::vector<InputSwizzlerMask>> {

    int GetSign() const {
        if (!RawSign())
            return +1;
        else
            return *RawSign();
    }

    const Identifier& GetIdentifier() const {
        return RawIdentifier();
    }

    bool HasIndexExpression() const {
        return RawIndex();
    }

    const IndexExpression& GetIndexExpression() const {
        return *RawIndex();
    }

    const std::vector<InputSwizzlerMask>& GetSwizzleMasks() const {
        return RawSwizzleMasks();
    }

private:
    const boost::optional<Sign>& RawSign() const {
        return boost::fusion::at_c<0>(boost::fusion::at_c<0>(*this));
    }

    const Identifier& RawIdentifier() const {
        return boost::fusion::at_c<1>(boost::fusion::at_c<0>(*this));
    }

    const boost::optional<IndexExpression>& RawIndex() const {
        return boost::fusion::at_c<1>(*this);
    }

    const std::vector<InputSwizzlerMask>& RawSwizzleMasks() const {
        return boost::fusion::at_c<2>(*this);
    }
};

using StatementLabel = std::string;

// TODO: Figure out why this cannot be a std::tuple...
struct StatementInstruction : boost::fusion::vector<Instruction::OpCode, std::vector<Expression>> {
    StatementInstruction() = default;

    StatementInstruction(const Instruction::OpCode& opcode) : boost::fusion::vector<Instruction::OpCode, std::vector<Expression>>(opcode, std::vector<Expression>()) {
    }

    const Instruction::OpCode& GetOpCode() const {
        return boost::fusion::at_c<0>(*this);
    }

    const std::vector<Expression>& GetArguments() const {
        return boost::fusion::at_c<1>(*this);
    }
};

using DeclarationConstant = boost::fusion::vector<std::string, Identifier, std::vector<float>>;
using DeclarationOutput   = boost::fusion::vector<std::string, Identifier, OutputRegisterInfo::Type>;
using DeclarationAlias    = boost::fusion::vector<std::string, Identifier>;

using StatementDeclaration = boost::variant<DeclarationConstant, DeclarationOutput, DeclarationAlias>;

struct ParserContext {
    // Maps known identifiers to an index to the controller's identifier list
    boost::spirit::qi::symbols<char, Identifier> identifiers;
};


struct Parser {
    using Iterator = std::string::iterator;

    Parser(const ParserContext& context);
    ~Parser();

    void Skip(Iterator& begin, Iterator end);

    bool ParseLabel(Iterator& begin, Iterator end, StatementLabel* label);

    bool ParseInstruction(Iterator& begin, Iterator end, StatementInstruction* instruction);

    bool ParseDeclaration(Iterator& begin, Iterator end, StatementDeclaration* declaration);

private:
    struct ParserImpl;
//    std::unique_ptr<ParserImpl> impl;
    ParserImpl* impl;
};
