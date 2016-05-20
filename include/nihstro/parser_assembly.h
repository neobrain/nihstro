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

#include <array>
#include <cstdint>
#include <memory>
#include <vector>
#include <ostream>

#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include "source_tree.h"

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

    friend std::string to_string(const Component& v) {
        std::stringstream ss;
        ss << v;
        return ss.str();
    }

    friend std::string to_string(const InputSwizzlerMask& v) {
        std::stringstream ss;
        ss << v;
        return ss.str();
    }
};

using Identifier = std::string;

// A sign, i.e. +1 or -1
using Sign = int;

struct IntegerWithSign {
    int sign;
    unsigned value;

    int GetValue() const {
        return sign * value;
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


struct Expression {
    struct SignedIdentifier {
        boost::optional<Sign> sign;
        Identifier identifier;
    } signed_identifier;

    boost::optional<IndexExpression> index;
    std::vector<InputSwizzlerMask> swizzle_masks;

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
        return static_cast<bool>(RawIndex());
    }

    const IndexExpression& GetIndexExpression() const {
        return *RawIndex();
    }

    const std::vector<InputSwizzlerMask>& GetSwizzleMasks() const {
        return RawSwizzleMasks();
    }

private:
    const boost::optional<Sign>& RawSign() const {
        return signed_identifier.sign;
    }

    const Identifier& RawIdentifier() const {
        return signed_identifier.identifier;
    }

    const boost::optional<IndexExpression>& RawIndex() const {
        return index;
    }

    const std::vector<InputSwizzlerMask>& RawSwizzleMasks() const {
        return swizzle_masks;
    }
};

struct ConditionInput {
    bool invert;
    Identifier identifier;
    boost::optional<InputSwizzlerMask> swizzler_mask;

    bool GetInvertFlag() const {
        return invert;
    }

    const Identifier& GetIdentifier() const {
        return identifier;
    }

    bool HasSwizzleMask() const {
        return static_cast<bool>(swizzler_mask);
    }

    const InputSwizzlerMask& GetSwizzleMask() const {
        return *swizzler_mask;
    }
};

struct Condition {
    ConditionInput input1;
    Instruction::FlowControlType::Op op;
    ConditionInput input2;

    const ConditionInput& GetFirstInput() const {
        return input1;
    }

    Instruction::FlowControlType::Op GetConditionOp() const {
        return op;
    }

    const ConditionInput& GetSecondInput() const {
        return input2;
    }
};

using StatementLabel = std::string;

struct StatementInstruction {
    OpCode opcode;
    std::vector<Expression> expressions;

    StatementInstruction() = default;

    // TODO: Obsolete constructor?
    StatementInstruction(const OpCode& opcode) : opcode(opcode) {
    }

    StatementInstruction(const OpCode& opcode, const std::vector<Expression> expressions) : opcode(opcode), expressions(expressions) {
    }

    const OpCode& GetOpCode() const {
        return opcode;
    }

    const std::vector<Expression>& GetArguments() const {
        return expressions;
    }
};
using FloatOpInstruction = StatementInstruction;

struct CompareInstruction {
    OpCode opcode;
    std::vector<Expression> arguments;
    std::vector<Instruction::Common::CompareOpType::Op> ops;

    const OpCode& GetOpCode() const {
        return opcode;
    }

    const Expression& GetSrc1() const {
        return arguments[0];
    }

    const Expression& GetSrc2() const {
        return arguments[1];
    }

    Instruction::Common::CompareOpType::Op GetOp1() const {
        return ops[0];
    }

    Instruction::Common::CompareOpType::Op GetOp2() const {
        return ops[1];
    }
};

struct FlowControlInstruction {
    OpCode opcode;
    std::string target_label;
    boost::optional<std::string> return_label;
    boost::optional<Condition> condition;

    const OpCode& GetOpCode() const {
        return opcode;
    }

    const std::string& GetTargetLabel() const {
        return target_label;
    }

    bool HasReturnLabel() const {
        return static_cast<bool>(return_label);
    }

    const std::string& GetReturnLabel() const {
        return *return_label;
    }

    bool HasCondition() const {
        return static_cast<bool>(condition);
    }

    const Condition& GetCondition() const {
        return *condition;
    }
};

struct SetEmitInstruction {
    OpCode opcode;
    unsigned vertex_id;

    struct Flags {
        boost::optional<bool> primitive_flag;
        boost::optional<bool> invert_flag;
    } flags;

    bool PrimitiveFlag() const {
        return flags.primitive_flag && *flags.primitive_flag;
    }

    bool InvertFlag() const {
        return flags.invert_flag && *flags.invert_flag;
    }
};

struct StatementDeclaration {
    std::string alias_name;
    Identifier identifier_start; /* aliased identifier (start register) */
    boost::optional<Identifier> identifier_end; /* aliased identifier (end register) */
    boost::optional<InputSwizzlerMask> swizzle_mask; // referring to the aliased identifier

    struct Extra {
        std::vector<float> constant_value;
        boost::optional<OutputRegisterInfo::Type> output_semantic;
    } extra;
};

struct ParserContext {
    // There currently is no context
};


struct Parser {
    using Iterator = SourceTreeIterator;

    Parser(const ParserContext& context);
    ~Parser();

    // Skip whitespaces, blank lines, and comments; returns number of line breaks skipped.
    unsigned Skip(Iterator& begin, Iterator end);

    // Skip to the next line
    void SkipSingleLine(Iterator& begin, Iterator end);

    // Parse alias declaration including line ending
    bool ParseDeclaration(Iterator& begin, Iterator end, StatementDeclaration* declaration);

    // Parse label declaration including line ending
    bool ParseLabel(Iterator& begin, Iterator end, StatementLabel* label);

    // Parse nothing but a single opcode
    bool ParseOpCode(Iterator& begin, Iterator end, OpCode* opcode);

    // Parse trival instruction including line ending
    bool ParseSimpleInstruction(Iterator& begin, Iterator end, OpCode* opcode);

    // Parse float instruction including line ending
    bool ParseFloatOp(Iterator& begin, Iterator end, FloatOpInstruction* content);

    // Parse compare instruction including line ending
    bool ParseCompare(Iterator& begin, Iterator end, CompareInstruction* content);

    // Parse flow control instruction including line ending
    bool ParseFlowControl(Iterator& begin, Iterator end, FlowControlInstruction* content);

    // Parse SetEmit instruction including line ending
    bool ParseSetEmit(Iterator& begin, Iterator end, SetEmitInstruction* content);

private:
    struct ParserImpl;
    std::unique_ptr<ParserImpl> impl;
};

} // namespace
