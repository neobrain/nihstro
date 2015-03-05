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

#include <boost/optional.hpp>
#include <array>
#include <vector>
#include <ostream>
#include <cstdint>

#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/at_c.hpp>

#include <boost/spirit/include/qi_symbols.hpp>

#include "shader_binary.h"
#include "shader_bytecode.h"

namespace nihstro {

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

using Identifier = std::string;

// A sign, i.e. +1 or -1
using Sign = int;

struct IntegerWithSign : boost::fusion::vector<int, unsigned int> {
    int GetValue() const {
        return boost::fusion::at_c<0>(*this) * boost::fusion::at_c<1>(*this);
    }
};

// Raw index + address register index
struct IndexExpression : std::vector<boost::variant<IntegerWithSign, Identifier>> {

    int GetCount() const {
        return this->size();
    }

    bool IsRawIndex(int arg) const {
        return (*this)[arg].which() == 0;
    }

    int GetRawIndex(int arg) const {
        return boost::get<IntegerWithSign>((*this)[arg]).GetValue();
    }

    bool IsAddressRegisterIdentifier(int arg) const {
        return (*this)[arg].which() == 1;
    }

    Identifier GetAddressRegisterIdentifier(int arg) const {
        return boost::get<Identifier>((*this)[arg]);
    }
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

struct ConditionInput : boost::fusion::vector<bool, Identifier, boost::optional<InputSwizzlerMask>> {
    bool GetInvertFlag() const {
        return boost::fusion::at_c<0>(*this);
    }

    const Identifier& GetIdentifier() const {
        return boost::fusion::at_c<1>(*this);
    }

    bool HasSwizzleMask() const {
        return boost::fusion::at_c<2>(*this);
    }

    const InputSwizzlerMask& GetSwizzleMask() const {
        return *boost::fusion::at_c<2>(*this);
    }

};

struct Condition : boost::fusion::vector<ConditionInput,
                                         Instruction::FlowControlType::Op,
                                         ConditionInput> {
    Condition() = default;

    const ConditionInput& GetFirstInput() const {
        return boost::fusion::at_c<0>(*this);
    }

    Instruction::FlowControlType::Op GetConditionOp() const {
        return boost::fusion::at_c<1>(*this);
    }

    const ConditionInput& GetSecondInput() const {
        return boost::fusion::at_c<2>(*this);
    }
};

using StatementLabel = std::string;

// TODO: Figure out why this cannot be a std::tuple...
struct StatementInstruction : boost::fusion::vector<OpCode, std::vector<Expression>> {
    StatementInstruction() = default;

    // TODO: Obsolete constructor?
    StatementInstruction(const OpCode& opcode) : boost::fusion::vector<OpCode, std::vector<Expression>>(opcode, std::vector<Expression>()) {
    }

    const OpCode& GetOpCode() const {
        return boost::fusion::at_c<0>(*this);
    }

    const std::vector<Expression>& GetArguments() const {
        return boost::fusion::at_c<1>(*this);
    }
};
using FloatOpInstruction = StatementInstruction;

struct CompareInstruction : boost::fusion::vector<OpCode, std::vector<Expression>, std::vector<Instruction::Common::CompareOpType::Op>> {
    CompareInstruction() = default;

    const OpCode& GetOpCode() const {
        return boost::fusion::at_c<0>(*this);
    }

    const Expression& GetSrc1() const {
        return boost::fusion::at_c<1>(*this)[0];
    }

    const Expression& GetSrc2() const {
        return boost::fusion::at_c<1>(*this)[1];
    }

    Instruction::Common::CompareOpType::Op GetOp1() const {
        return boost::fusion::at_c<2>(*this)[0];
    }

    Instruction::Common::CompareOpType::Op GetOp2() const {
        return boost::fusion::at_c<2>(*this)[1];
    }
};

struct FlowControlInstruction : boost::fusion::vector<OpCode,
                                                      std::string /*target_label*/,
                                                      boost::optional<std::string> /*return_label*/,
                                                      boost::optional<Condition>> {
    using ParentType = boost::fusion::vector<OpCode, std::string, boost::optional<std::string>, boost::optional<Condition>>;

    FlowControlInstruction() = default;

    FlowControlInstruction(const ParentType& obj) : boost::fusion::vector<OpCode, std::string, boost::optional<std::string>, boost::optional<Condition>>(obj) {};
    const OpCode& GetOpCode() const {
        return boost::fusion::at_c<0>(*this);
    }

    const std::string& GetTargetLabel() const {
        return boost::fusion::at_c<1>(*this);
    }

    bool HasReturnLabel() const {
        return boost::fusion::at_c<2>(*this);
    }

    const std::string& GetReturnLabel() const {
        return *boost::fusion::at_c<2>(*this);
    }

    bool HasCondition() const {
        return boost::fusion::at_c<3>(*this);
    }

    const Condition& GetCondition() const {
        return *boost::fusion::at_c<3>(*this);
    }

};

using StatementDeclaration = boost::fusion::vector<std::string /* alias name */,
                                                   Identifier /* aliased identifier (start register) */,
                                                   boost::optional<Identifier> /* aliased identifier (end register) */,
                                                   boost::optional<InputSwizzlerMask> /* swizzle mask */,
                                                   boost::fusion::vector<std::vector<float> /* constant value */,
                                                                         boost::optional<OutputRegisterInfo::Type> /* output semantic */>>;


struct ParserContext {
    // There currently is no context
};


struct Parser {
    using Iterator = std::string::iterator;

    Parser(const ParserContext& context);
    ~Parser();

    void Skip(Iterator& begin, Iterator end);

    void SkipSingleLine(Iterator& begin, Iterator end);

    bool ParseDeclaration(Iterator& begin, Iterator end, StatementDeclaration* declaration);

    bool ParseLabel(Iterator& begin, Iterator end, StatementLabel* label);

    bool ParseSimpleInstruction(Iterator& begin, Iterator end, OpCode* opcode);

    bool ParseFloatOp(Iterator& begin, Iterator end, FloatOpInstruction* content);

    bool ParseCompare(Iterator& begin, Iterator end, CompareInstruction* content);

    bool ParseFlowControl(Iterator& begin, Iterator end, FlowControlInstruction* content);

private:
    struct ParserImpl;
    std::unique_ptr<ParserImpl> impl;
};

} // namespace
