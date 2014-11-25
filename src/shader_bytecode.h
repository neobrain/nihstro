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

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>

#include "bit_field.h"

#pragma pack(1)

union Instruction {
    enum class OpCode : uint32_t {
        ADD     = 0x00,
        DP3     = 0x01,
        DP4     = 0x02,

        MUL     = 0x08,

        MAX     = 0x0C,
        MIN     = 0x0D,
        RCP     = 0x0E,
        RSQ     = 0x0F,

        ARL     = 0x12,   // Address Register Load
        MOV     = 0x13,

        NOP     = 0x21,
        END     = 0x22,
        BREAKC  = 0x23,

        CALL    = 0x24,
        CALLC   = 0x25,
        CALLU   = 0x26,
        IFU     = 0x27,
        IFC     = 0x28,
        FOR     = 0x29,
        EMIT    = 0x2A,
        SETEMIT = 0x2B,
        JMPC    = 0x2C,

        CMP     = 0x2E, // LSB opcode bit ignored

        MAD     = 0x38, // lower 3 opcode bits ignored
    };

    enum RegisterType {
        Input,
        Output,
        Temporary,
        FloatUniform,
        Address,
        Unknown
    };

    enum class OpCodeType {
        Trivial,            // 3dbrew format 0
        Arithmetic,         // 3dbrew format 1
        ArithmeticInversed, // 3dbrew format 1c
        Conditional,        // 3dbrew format 2
        UniformFlowControl, // 3dbrew format 3
        SetEmit,            // 3dbrew format 4
        MultiplyAdd,        // 3dbrew format 5
        Unknown
    };

    struct OpCodeInfo {
        OpCodeType type;

        enum : uint32_t {
            OpDesc = 1,
            Src1   = 2,
            Src2   = 4,
            Idx    = 8,
            Dest   = 16,
            OneArgument = OpDesc | Src1 | Idx | Dest,
            TwoArguments = OneArgument | Src2,
            AddressRegisterLoad,
        };

        enum : uint32_t {
            Dst                 = 1,
            Num                 = 2,
            JustDstNum          = Dst | Num,
            Op                  = 4,
            NegY                = 8,
            NegX                = 16,
            JustCondition       = Op | NegY | NegX,
            JustConditionAndDst = JustCondition | Dst,
            Full                = JustConditionAndDst | Num
        };

        enum : uint32_t {
            FullAndBool,
            SimpleAndInt,
        };

        uint32_t subtype;

        std::string name;
    };

    static std::string GetRegisterName(RegisterType type) {
        std::map<RegisterType, std::string> map = {
            { Input, "v" },
            { Output, "o" },
            { Temporary, "r" },
            { FloatUniform, "c" },
            { Unknown, "u" },
        };
        return map[type];
    }

    uint32_t hex;

    struct : BitField<0x1a, 0x6, OpCode> {
        OpCodeInfo GetInfo() const {
            std::map<OpCode, OpCodeInfo> map = {

                { OpCode::ADD,     { OpCodeType::Arithmetic,         OpCodeInfo::TwoArguments,        "add" } },
                { OpCode::DP3,     { OpCodeType::Arithmetic,         OpCodeInfo::TwoArguments,        "dp3" } },
                { OpCode::DP4,     { OpCodeType::Arithmetic,         OpCodeInfo::TwoArguments,        "dp4" } },
                { OpCode::MUL,     { OpCodeType::Arithmetic,         OpCodeInfo::TwoArguments,        "mul" } },
                { OpCode::MAX,     { OpCodeType::Arithmetic,         OpCodeInfo::TwoArguments,        "max" } },
                { OpCode::MIN,     { OpCodeType::Arithmetic,         OpCodeInfo::TwoArguments,        "min" } },
                { OpCode::RCP,     { OpCodeType::Arithmetic,         OpCodeInfo::OneArgument,         "rcp" } },
                { OpCode::RSQ,     { OpCodeType::Arithmetic,         OpCodeInfo::OneArgument,         "rsq" } },
                { OpCode::ARL,     { OpCodeType::Arithmetic,         OpCodeInfo::AddressRegisterLoad, "arl" } },
                { OpCode::MOV,     { OpCodeType::Arithmetic,         OpCodeInfo::OneArgument,         "mov" } },
                { OpCode::NOP,     { OpCodeType::Trivial,            0,                               "nop" } },
                { OpCode::END,     { OpCodeType::Trivial,            0,                               "end" } },
                { OpCode::BREAKC,  { OpCodeType::Conditional,        OpCodeInfo::JustCondition,       "breakc" } },
                { OpCode::CALL,    { OpCodeType::Conditional,        OpCodeInfo::JustDstNum,          "call" } },
                { OpCode::CALLC,   { OpCodeType::Conditional,        OpCodeInfo::Full,                "callc" } },
                { OpCode::CALLU,   { OpCodeType::UniformFlowControl, OpCodeInfo::FullAndBool,         "callu" } },
                { OpCode::IFU,     { OpCodeType::UniformFlowControl, OpCodeInfo::FullAndBool,         "ifu" } },
                { OpCode::IFC,     { OpCodeType::Conditional,        OpCodeInfo::Full,                "ifc" } },
                { OpCode::FOR,     { OpCodeType::UniformFlowControl, OpCodeInfo::SimpleAndInt,        "for" } },
                { OpCode::EMIT,    { OpCodeType::Trivial,            0,                               "emit" } },
                { OpCode::SETEMIT, { OpCodeType::SetEmit,            0,                               "setemit" } },
                { OpCode::JMPC,    { OpCodeType::Conditional,        OpCodeInfo::JustConditionAndDst, "jmpc" } },
                { OpCode::CMP,     { OpCodeType::ArithmeticInversed, 0,                               "cmp" } }, // TODO: Wrong type
                { OpCode::MAD,     { OpCodeType::MultiplyAdd,        0,                               "mad" } },
            };
            auto it = map.find(*this);
            if (it == map.end())
                return { OpCodeType::Unknown, 0, std::string("UNK") + std::to_string(static_cast<int>(this->Value())) };
            else
                return it->second;
        }
    } opcode;

    // General notes:
    //
    // When two input registers are used, one of them uses a 5-bit index while the other
    // one uses a 7-bit index. This is because at most one floating point uniform may be used
    // as an input.


    // Format used e.g. by arithmetic instructions and comparisons
    union {
        BitField<0x00, 0x7, uint32_t> operand_desc_id;

        template<class BitFieldType>
        struct SourceRegister : BitFieldType {

            RegisterType GetRegisterType() const {
                if (this->Value() < 0x10)
                    return Input;
                else if (this->Value() < 0x20)
                    return Temporary;
                else
                    return FloatUniform;
            }

            int GetIndex() const {
                if (GetRegisterType() == Input)
                    return this->Value();
                else if (GetRegisterType() == Temporary)
                    return this->Value() - 0x10;
                else if (GetRegisterType() == FloatUniform)
                    return this->Value() - 0x20;
            }

            void InitializeFromTypeAndIndex(RegisterType type, int index) {
                if (type == Input)
                    this->Assign(index);
                else if (type == Temporary)
                    this->Assign(index + 0x10);
                else if (type == FloatUniform)
                    this->Assign(index + 0x20);
                else {
                    // TODO: Should throw an exception or something.
                }
            }

            std::string GetName() const {
                return GetRegisterName(GetRegisterType()) + std::to_string(GetIndex());
            }
        };

        SourceRegister<BitField<0x07, 0x5, uint32_t>> src2;
        SourceRegister<BitField<0x0c, 0x7, uint32_t>> src1;

        // Address register value is used for relative addressing of src1
        BitField<0x13, 0x2, uint32_t> address_register_index;

        std::string AddressRegisterName() const {
            if (address_register_index == 0) return "";
            else if (address_register_index == 1) return "a0";
            else if (address_register_index == 2) return "a1";
            else /*if (address_register_index == 3)*/ return "lcnt";
        }

        struct : BitField<0x15, 0x5, uint32_t>
        {
            RegisterType GetRegisterType() const {
                if (Value() < 0x8)
                    return Output;
                else if (Value() < 0x10)
                    return Unknown;
                else
                    return Temporary;
            }

            int GetIndex() const {
                if (GetRegisterType() == Output)
                    return this->Value();
                else if (GetRegisterType() == Temporary)
                    return this->Value() - 0x10;
                else // if (GetRegisterType() == FloatUniform)
                    return this->Value() - 0x20;
            }

            void InitializeFromTypeAndIndex(RegisterType type, int index) {
                if (type == Output)
                    this->Assign(index);
                else if (type == Temporary)
                    this->Assign(index + 0x10);
                else if (type == FloatUniform)
                    this->Assign(index + 0x20);
                else {
                    // TODO: Should throw an exception or something.
                }
            }

            std::string GetName() const {
                return GetRegisterName(GetRegisterType()) + std::to_string(GetIndex());
            }
        } dest;
    } common;

    union {
        enum Op : uint32_t {
            Or    = 0,
            And   = 1,
            JustX = 2,
            JustY = 3
        };

        BitField<0x00, 0x8, uint32_t> num_instructions;
        BitField<0x0a, 0xc, uint32_t> dest_offset;
        BitField<0x16, 0x2, Op> op;
        BitField<0x18, 0x1, uint32_t> negy;
        BitField<0x19, 0x1, uint32_t> negx;
    } conditional;

    // Format used for flow control instructions ("if")
    // TODO: Likely obsolete.
    union {
        BitField<0x00, 0x8, uint32_t> num_instructions;
        BitField<0x0a, 0xc, uint32_t> offset_words;
    } flow_control;
};
static_assert(sizeof(Instruction) == 0x4, "Incorrect structure size");
static_assert(std::is_standard_layout<Instruction>::value, "Structure does not have standard layout");

union SwizzlePattern {
    uint32_t hex;

    enum class Selector : uint32_t {
        x = 0,
        y = 1,
        z = 2,
        w = 3
    };

    Selector GetSelectorSrc1(int comp) const {
        Selector selectors[] = {
            src1_selector_0, src1_selector_1, src1_selector_2, src1_selector_3
        };
        return selectors[comp];
    }

    Selector GetSelectorSrc2(int comp) const {
        Selector selectors[] = {
            src2_selector_0, src2_selector_1, src2_selector_2, src2_selector_3
        };
        return selectors[comp];
    }

    void SetSelectorSrc1(int comp, Selector value) {
        if (comp == 0)
            src1_selector_0 = value;
        else if (comp == 1)
            src1_selector_1 = value;
        else if (comp == 2)
            src1_selector_2 = value;
        else if (comp == 3)
            src1_selector_3 = value;
        else
            throw std::out_of_range("comp needs to be smaller than 4");
    }

    void SetSelectorSrc2(int comp, Selector value) {
        if (comp == 0)
            src2_selector_0 = value;
        else if (comp == 1)
            src2_selector_1 = value;
        else if (comp == 2)
            src2_selector_2 = value;
        else if (comp == 3)
            src2_selector_3 = value;
        else
            throw std::out_of_range("comp needs to be smaller than 4");
    }

    std::string SelectorToString(bool src2) const {
        std::map<Selector, std::string> map = {
            { Selector::x, "x" },
            { Selector::y, "y" },
            { Selector::z, "z" },
            { Selector::w, "w" }
        };
        std::string ret;
        for (int i = 0; i < 4; ++i) {
            ret += map.at(src2 ? GetSelectorSrc2(i) : GetSelectorSrc1(i));
        }
        return ret;
    }

    bool DestComponentEnabled(unsigned int i) const {
        if (i >= 4)
            throw std::out_of_range("index needs to be smaller than 4");

        return (dest_mask & (0x8 >> i));
    }

    void SetDestComponentEnabled(unsigned int i, bool enabled) {
        int mask = 0xffff & (0x8 >> i);
        dest_mask = (dest_mask & ~mask) | (enabled * mask);
    }

    std::string DestMaskToString() const {
        std::string ret;
        for (int i = 0; i < 4; ++i) {
            if (!DestComponentEnabled(i))
                ret += "_";
            else
                ret += "xyzw"[i];
        }
        return ret;
    }

    // Components of "dest" that should be written to: LSB=dest.w, MSB=dest.x
    BitField< 0, 4, uint32_t> dest_mask;

    BitField< 4, 1, uint32_t> negate_src1;
    BitField< 5, 2, Selector> src1_selector_3;
    BitField< 7, 2, Selector> src1_selector_2;
    BitField< 9, 2, Selector> src1_selector_1;
    BitField<11, 2, Selector> src1_selector_0;

    BitField<13, 1, uint32_t> negate_src2;
    BitField<14, 2, Selector> src2_selector_3;
    BitField<16, 2, Selector> src2_selector_2;
    BitField<18, 2, Selector> src2_selector_1;
    BitField<20, 2, Selector> src2_selector_0;

    BitField<22, 1, uint32_t> negate_src3;
    BitField<23, 2, Selector> src3_selector_3;
    BitField<25, 2, Selector> src3_selector_2;
    BitField<27, 2, Selector> src3_selector_1;
    BitField<29, 2, Selector> src3_selector_0;
};
static_assert(sizeof(SwizzlePattern) == 0x4, "Incorrect structure size");


#pragma pack()
