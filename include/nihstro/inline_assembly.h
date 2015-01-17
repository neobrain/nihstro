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

#include <algorithm>
#include <array>
#include <initializer_list>
#include <vector>

#include "shader_binary.h"
#include "shader_bytecode.h"

namespace nihstro {

struct ShaderBinary {
    std::vector<Instruction> program;
    std::vector<SwizzlePattern> swizzle_table;
};

struct DestRegisterOrTemporary : public DestRegister {
    DestRegisterOrTemporary(const DestRegister& oth) : DestRegister(oth) {}
    DestRegisterOrTemporary(const SourceRegister& oth) : DestRegister(DestRegister::FromTypeAndIndex(oth.GetRegisterType(), oth.GetIndex())) {
        if (oth.GetRegisterType() != RegisterType::Temporary)
            throw "Invalid source register used as output";
    }
};

struct InlineAsm {
    struct DestMask {
        DestMask(const std::string& mask) {
            static const std::map<std::string,uint32_t> valid_masks {
                { "x", 8 }, { "y", 4 }, { "z", 2 }, { "w", 1 },
                { "xy", 12 }, { "xz", 10 }, { "xw", 9 },
                { "yz", 6 }, { "yw", 5 }, { "zw", 3 },
                { "xyz", 14 }, { "xyw", 13 }, { "xzw", 11 }, { "yzw", 7 },
                { "xyzw", 15 }, { "", 15 }
            };

            dest_mask = valid_masks.at(mask);
        }

        DestMask(const char* mask) : DestMask(std::string(mask)) {}

        DestMask(const DestMask&) = default;

        uint32_t dest_mask;
    };

    struct SwizzleMask {
        SwizzleMask(const std::string& swizzle) : negate(false) {
            selectors[0] = SwizzlePattern::Selector::x;
            selectors[1] = SwizzlePattern::Selector::y;
            selectors[2] = SwizzlePattern::Selector::z;
            selectors[3] = SwizzlePattern::Selector::w;

            if (swizzle.length() == 0)
                return;

            if (swizzle.length() > 5) {
                throw "Invalid swizzle mask";
            }

            int index = 0;

            if (swizzle[index] == '-') {
                negate = true;
            } else if (swizzle[index] == '+') {
                index++;
            }

            for (int i = 0; i < 4; ++i) {
                if (swizzle.length() <= index + i)
                    return;

                switch (swizzle[index + i]) {
                case 'x':  selectors[i] = SwizzlePattern::Selector::x;  break;
                case 'y':  selectors[i] = SwizzlePattern::Selector::y;  break;
                case 'z':  selectors[i] = SwizzlePattern::Selector::z;  break;
                case 'w':  selectors[i] = SwizzlePattern::Selector::w;  break;
                default:
                    throw "Invalid swizzle mask";
                }
            }
        }

        SwizzleMask(const char* swizzle) : SwizzleMask(std::string(swizzle)) {}

        SwizzleMask(const SwizzleMask&) = default;

        SwizzlePattern::Selector selectors[4];
        bool negate;
    };

    enum Type {
        Regular,
        Else,
        EndIf,
        EndLoop,
        Label
    } type;

    InlineAsm(Type type) : type(type) {
    }

    InlineAsm(OpCode opcode) : type(Regular) {
        if (opcode.GetInfo().type != OpCode::Type::Trivial) {
            throw "Invalid opcode used with zero arguments";
        }
        full_instruction.instr.opcode = opcode;
    }

    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1, const SwizzleMask swizzle_src1 = SwizzleMask{""}) : type(Regular) {
        Instruction& instr = full_instruction.instr;
        instr.hex = 0;
        instr.opcode = opcode;
        SwizzlePattern& swizzle = full_instruction.swizzle;
        swizzle.hex = 0;

        switch(opcode.GetInfo().type) {
        case OpCode::Type::Arithmetic:
            // TODO: Assert valid inputs, considering the field width!
            instr.common.dest = dest;
            instr.common.src1 = src1;
            swizzle.negate_src1 = swizzle_src1.negate;

            swizzle.dest_mask = dest_mask.dest_mask;
            for (int i = 0; i < 4; ++i) {
                swizzle.SetSelectorSrc1(i, swizzle_src1.selectors[i]);
            }
            break;

        default:
            throw "Unknown inline assmembler command";
        }
    }

    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1, const SwizzleMask& swizzle_src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2 = "") : type(Regular) {
        Instruction& instr = full_instruction.instr;
        instr.hex = 0;
        instr.opcode = opcode;
        SwizzlePattern& swizzle = full_instruction.swizzle;
        swizzle.hex = 0;

        switch(opcode.GetInfo().type) {
        case OpCode::Type::Arithmetic:
            // TODO: Assert valid inputs, considering the field width!
            instr.common.dest = dest;
            instr.common.src1 = src1;
            instr.common.src2 = src2;
            swizzle.negate_src1 = swizzle_src1.negate;
            swizzle.negate_src2 = swizzle_src2.negate;

            swizzle.dest_mask = dest_mask.dest_mask;
            for (int i = 0; i < 4; ++i) {
                swizzle.SetSelectorSrc1(i, swizzle_src1.selectors[i]);
                swizzle.SetSelectorSrc2(i, swizzle_src2.selectors[i]);
            }
            break;

        default:
            throw "Unknown inline assembler command";
        }
    }

    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1, const SwizzleMask& swizzle_src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = {""}) : type(Regular) {
        Instruction& instr = full_instruction.instr;
        instr.hex = 0;
        instr.opcode = opcode;
        SwizzlePattern& swizzle = full_instruction.swizzle;
        swizzle.hex = 0;

        switch(opcode.GetInfo().type) {
        case OpCode::Type::MultiplyAdd:
            // TODO: Assert valid inputs, considering the field width!
            instr.mad.dest = dest;
            instr.mad.src1 = src1;
            instr.mad.src2 = src2;
            instr.mad.src3 = src3;
            full_instruction.swizzle.negate_src1 = swizzle_src1.negate;
            full_instruction.swizzle.negate_src2 = swizzle_src2.negate;
            full_instruction.swizzle.negate_src3 = swizzle_src3.negate;

            swizzle.dest_mask = dest_mask.dest_mask;
            for (int i = 0; i < 4; ++i) {
                full_instruction.swizzle.SetSelectorSrc1(i, swizzle_src1.selectors[i]);
                full_instruction.swizzle.SetSelectorSrc2(i, swizzle_src2.selectors[i]);
            }
            break;

        default:
            throw "Unknown inline assembler command";
        }
    }

    // Convenience constructors with implicit swizzle mask
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1, const SwizzleMask swizzle_src1 = SwizzleMask{ "" }) : InlineAsm(opcode, dest, "", src1, swizzle_src1) {}

    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1, const SwizzleMask& swizzle_src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2 = "") : InlineAsm(opcode, dest, "", src1, swizzle_src1, src2, swizzle_src2) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1, SourceRegister src2,
              const SwizzleMask& swizzle_src2 = "") : InlineAsm(opcode, dest, dest_mask, src1, "", src2, swizzle_src2) {}

    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1, const SwizzleMask& swizzle_src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = "") : InlineAsm(opcode, dest, "", src1, swizzle_src1, src2, swizzle_src2, src3, swizzle_src3) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = "") : InlineAsm(opcode, dest, dest_mask, src1, "", src2, swizzle_src2, src3, swizzle_src3) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1, const SwizzleMask& swizzle_src1,
              SourceRegister src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = "") : InlineAsm(opcode, dest, dest_mask, src1, swizzle_src1, src2, "", src3, swizzle_src3) {}

    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = "") : InlineAsm(opcode, dest, "", src1, "", src2, swizzle_src2, src3, swizzle_src3) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1, const SwizzleMask& swizzle_src1,
              SourceRegister src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = "") : InlineAsm(opcode, dest, "", src1, swizzle_src1, src2, "", src3, swizzle_src3) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1,
              SourceRegister src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = "") : InlineAsm(opcode, dest, dest_mask, src1, "", src2, "", src3, swizzle_src3) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1,
              SourceRegister src2,
              SourceRegister src3, const SwizzleMask& swizzle_src3 = "") : InlineAsm(opcode, dest, "", src1, "", src2, "", src3, swizzle_src3) {}

    // TODO: Group this into a union once MSVC supports unrestricted unions!
    struct {
        Instruction instr;
        SwizzlePattern swizzle;
    } full_instruction;

    std::string name;

    static size_t FindSwizzlePattern(const SwizzlePattern& pattern, std::vector<SwizzlePattern>& swizzle_table) {
        auto it = std::find_if(swizzle_table.begin(), swizzle_table.end(), [&](const SwizzlePattern& candidate) { return candidate.hex == pattern.hex; });
        size_t ret = std::distance(swizzle_table.begin(), it);

        if (it == swizzle_table.end())
            swizzle_table.push_back(pattern);

        return ret;
    }

    static const ShaderBinary CompileToRawBinary(std::initializer_list<InlineAsm> code) {
        ShaderBinary binary;
        int index = 0;
        for (auto command : code) {
            switch (command.type) {
            case Regular:
            {
                auto& instr = command.full_instruction.instr;

                switch (instr.opcode.Value().GetInfo().type) {
                case OpCode::Type::Trivial:
                    break;

                case OpCode::Type::Arithmetic:
                    instr.common.operand_desc_id = FindSwizzlePattern(command.full_instruction.swizzle, binary.swizzle_table);
                    break;

                default:
                    throw "Unknown instruction";
                }

                binary.program.push_back(command.full_instruction.instr);
                break;
            }

            default:
                throw "Unknown type";
            }

            index++;
        }
        return binary;
    }

    static const size_t CompiledShbinSize(std::initializer_list<InlineAsm> code) {
        size_t size = 0;
        size += sizeof(DVLBHeader);
        size += sizeof(DVLPHeader);
        size += sizeof(uint32_t) + sizeof(DVLEHeader); // Currently only one DVLE is supported

        for (const auto& command : code) {
            switch (command.type) {
            case Regular:
                size += sizeof(Instruction);
                size += sizeof(SwizzleInfo);
                break;
            }
        }

        return size;
    }

    static const std::vector<uint8_t> CompileToShbin(std::initializer_list<InlineAsm> code) {
        std::vector<uint8_t> ret(CompiledShbinSize(code));

        ShaderBinary bin = CompileToRawBinary(code);

        struct {
            DVLBHeader header;
            uint32_t dvle_offset;
        } *dvlb = (decltype(dvlb))ret.data();
        dvlb->header.magic_word = DVLBHeader::MAGIC_WORD;
        dvlb->header.num_programs = 1;

        unsigned dvlp_offset = sizeof(*dvlb);
        DVLPHeader* dvlp = (DVLPHeader*)&ret.data()[dvlp_offset];
        dvlp->magic_word = DVLPHeader::MAGIC_WORD;

        unsigned dvle_offset = dvlb->dvle_offset = dvlp_offset + sizeof(DVLPHeader);
        DVLEHeader* dvle = (DVLEHeader*)&ret.data()[dvle_offset];
        dvle->magic_word = DVLEHeader::MAGIC_WORD;
        dvlb->dvle_offset = dvle_offset;

        unsigned binary_offset = dvle_offset + sizeof(DVLEHeader);
        dvlp->binary_offset = binary_offset - dvlp_offset;
        dvlp->binary_size_words = bin.program.size();
        std::copy(bin.program.begin(), bin.program.end(), (Instruction*)&ret.data()[binary_offset]);

        unsigned swizzle_table_offset = binary_offset + bin.program.size() * sizeof(Instruction);
        dvlp->swizzle_info_offset = swizzle_table_offset - dvlp_offset;
        dvlp->swizzle_info_num_entries = bin.swizzle_table.size();
        SwizzleInfo* swizzle_table_ptr = (SwizzleInfo*)&ret.data()[swizzle_table_offset];
        for (const auto& swizzle : bin.swizzle_table) {
            swizzle_table_ptr->pattern = swizzle;
            swizzle_table_ptr->unknown = 0;
            swizzle_table_ptr++;
        }

        return ret;
    }
};

} // namespace
