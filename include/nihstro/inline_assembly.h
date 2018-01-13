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

#include "bit_field.h"
#include "float24.h"
#include "shader_binary.h"
#include "shader_bytecode.h"

namespace nihstro {

struct ShaderBinary {
    std::vector<Instruction> program;
    std::vector<SwizzlePattern> swizzle_table;
    std::vector<OutputRegisterInfo> output_table;
    std::vector<ConstantInfo> constant_table;
    std::vector<char> symbol_table;
    std::vector<UniformInfo> uniform_table;
};

struct DestRegisterOrTemporary : public DestRegister {
    DestRegisterOrTemporary(const DestRegister& oth) : DestRegister(oth) {}
    DestRegisterOrTemporary(const SourceRegister& oth) : DestRegister(DestRegister::FromTypeAndIndex(oth.GetRegisterType(), oth.GetIndex())) {
        if (oth.GetRegisterType() != RegisterType::Temporary)
            throw "Invalid source register used as output";
    }
};

struct InlineAsm {
    enum RelativeAddress {
        None,
        A1,
        A2,
        AL
    };

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
        Output,
        Constant,
        Uniform,
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

    InlineAsm(OpCode opcode, int src) : type(Regular) {
        switch (opcode.EffectiveOpCode()) {
        case OpCode::Id::LOOP:
//            if (src.GetRegisterType() != RegisterType::IntUniform)
//                throw "LOOP argument must be an integer register!";

            reg_id = src;
            full_instruction.instr.hex = 0;
            full_instruction.instr.opcode = opcode;
            break;

        default:
            throw "Unknown opcode argument";
        }
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
              SourceRegister src2, const SwizzleMask& swizzle_src2 = "", RelativeAddress addr = None) : type(Regular) {
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
            instr.common.address_register_index = addr;
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
              SourceRegister src1, const SwizzleMask& swizzle_src1 = "") : InlineAsm(opcode, dest, "", src1, swizzle_src1) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1, const SwizzleMask& swizzle_src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2 = "") : InlineAsm(opcode, dest, "", src1, swizzle_src1, src2, swizzle_src2) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest, const DestMask& dest_mask,
              SourceRegister src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2 = "") : InlineAsm(opcode, dest, dest_mask, src1, "", src2, swizzle_src2) {}
    InlineAsm(OpCode opcode, DestRegisterOrTemporary dest,
              SourceRegister src1,
              SourceRegister src2, const SwizzleMask& swizzle_src2 = "") : InlineAsm(opcode, dest, "", src1, "", src2, swizzle_src2) {}
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

    static InlineAsm DeclareOutput(DestRegister reg, OutputRegisterInfo::Type semantic) {
        if (reg.GetRegisterType() != RegisterType::Output)
            throw "Invalid register used to declare output";

        InlineAsm ret(Output);
        ret.output_semantic = semantic;
        ret.reg_id = reg.GetIndex();
        return ret;
    }

    static InlineAsm DeclareConstant(SourceRegister reg, float x, float y, float z, float w) {
        if (reg.GetRegisterType() != RegisterType::FloatUniform)
            throw "Invalid source register used to declare shader constant";

        InlineAsm ret(Constant);
        ret.value[0] = x;
        ret.value[1] = y;
        ret.value[2] = z;
        ret.value[3] = w;
        ret.constant_type = ConstantInfo::Float;
        ret.reg_id = reg.GetIndex();
        return ret;
    }

    static InlineAsm DeclareUniform(SourceRegister reg_first, SourceRegister reg_last, const std::string& name) {
        InlineAsm ret(Uniform);
        ret.reg_id = reg_first.GetIndex();
        ret.reg_id_last = reg_last.GetIndex();
        ret.name = name;
        return ret;
    }

    // TODO: Group this into a union once MSVC supports unrestricted unions!
    struct {
        Instruction instr;
        SwizzlePattern swizzle;
    } full_instruction;

    std::string name;

    unsigned reg_id;
    unsigned reg_id_last;

    OutputRegisterInfo::Type output_semantic;

    ConstantInfo::Type constant_type;
    float value[4];

    static size_t FindSwizzlePattern(const SwizzlePattern& pattern, std::vector<SwizzlePattern>& swizzle_table) {
        auto it = std::find_if(swizzle_table.begin(), swizzle_table.end(), [&](const SwizzlePattern& candidate) { return candidate.hex == pattern.hex; });
        size_t ret = std::distance(swizzle_table.begin(), it);

        if (it == swizzle_table.end())
            swizzle_table.push_back(pattern);

        return ret;
    }

    static const ShaderBinary CompileToRawBinary(std::initializer_list<InlineAsm> code_) {
        ShaderBinary binary;
        std::vector<InlineAsm> code(code_);
        for (int i = 0; i < code.size(); ++i) {
            auto command = code[i];

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

                case OpCode::Type::UniformFlowControl:
                    if (instr.opcode.Value().EffectiveOpCode() == OpCode::Id::LOOP) {
                        instr.flow_control.int_uniform_id = command.reg_id;
                        instr.flow_control.dest_offset = binary.program.size();

//                        std::cout << "Started at "<< binary.program.size() << " <-> "<<i <<std::endl;
                        // TODO: Handle nested LOOPs
                        for (int i2 = i + 1; i2 < code.size(); ++i2) {
                            if (code[i2].type == Regular) {
  //                              std::cout << "went at "<< i2 << std::endl;
                                instr.flow_control.dest_offset = instr.flow_control.dest_offset + 1;
                            } else if (code[i2].type == EndLoop) {
                                break;
                            }

                            if (i2 == code.size() - 1) {
                                throw "No closing EndLoop directive found";
                            }
                        }
                    } else {
                        throw "Unknown flow control instruction";
                    }
                    break;

                default:
                    throw "Unknown instruction";
                }

                binary.program.push_back(command.full_instruction.instr);
                break;
            }

            case Output:
            {
                OutputRegisterInfo output;
                output.type = command.output_semantic;
                output.id = command.reg_id;
                output.component_mask = 0xF; // TODO: Make configurable
                binary.output_table.push_back(output);
                break;
            }

            case Constant:
            {
                ConstantInfo constant;
                constant.type = command.constant_type;
                constant.regid = command.reg_id;

                switch (command.constant_type) {
                case ConstantInfo::Float:
                    constant.f.x = to_float24(command.value[0]);
                    constant.f.y = to_float24(command.value[1]);
                    constant.f.z = to_float24(command.value[2]);
                    constant.f.w = to_float24(command.value[3]);
                    break;

                default:
                    throw "Unknown constant type";
                }
                binary.constant_table.push_back(constant);
                break;
            }

            case Uniform:
            {
                UniformInfo uniform;
                uniform.basic.symbol_offset = binary.symbol_table.size();
                uniform.basic.reg_start = command.reg_id + 16; // TODO: Hardcoded against float uniforms
                uniform.basic.reg_end = command.reg_id_last + 16;
                binary.uniform_table.push_back(uniform);

                std::copy(command.name.begin(), command.name.end(), std::back_inserter(binary.symbol_table));
                binary.symbol_table.push_back('\0');

                break;
            }

            case EndLoop:
                break;

            default:
                throw "Unknown type";
            }
        }
        return binary;
    }

    // Overestimates the actual size
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

            case Output:
                size += sizeof(OutputRegisterInfo);
                break;

            case Constant:
                size += sizeof(ConstantInfo);
                break;

            case Uniform:
                size += command.name.size() + 1;
                size += sizeof(UniformInfo);
                break;

            case EndLoop:
                break;

            default:
                throw "Unknown command type";
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

        unsigned output_table_offset = swizzle_table_offset + bin.swizzle_table.size() * sizeof(SwizzleInfo);
        OutputRegisterInfo* output_table_ptr = (OutputRegisterInfo*)&ret.data()[output_table_offset];
        for (const auto& output : bin.output_table) {
            *output_table_ptr = output;
            output_table_ptr++;
        }
        dvle->output_register_table_offset = output_table_offset - dvle_offset;
        dvle->output_register_table_size = bin.output_table.size();

        unsigned constant_table_offset = output_table_offset + bin.output_table.size() * sizeof(OutputRegisterInfo);
        ConstantInfo* constant_table_ptr = (ConstantInfo*)&ret.data()[constant_table_offset];
        for (const auto& constant : bin.constant_table) {
            *constant_table_ptr = constant;
            constant_table_ptr++;
        }
        dvle->constant_table_offset = constant_table_offset - dvle_offset;
        dvle->constant_table_size = bin.constant_table.size();

        // TODO: UniformTable spans more than the written data.. fix this design issue :/
        unsigned uniform_table_offset = constant_table_offset + bin.constant_table.size() * sizeof(ConstantInfo);
        uint64_t* uniform_table_ptr = (uint64_t*)&ret.data()[uniform_table_offset];
        for (const auto& uniform : bin.uniform_table) {
            *uniform_table_ptr = reinterpret_cast<const uint64_t&>(uniform.basic);
            uniform_table_ptr++;
        }
        dvle->uniform_table_offset = uniform_table_offset - dvle_offset;
        dvle->uniform_table_size = bin.uniform_table.size();

        unsigned symbol_table_offset = uniform_table_offset + bin.uniform_table.size() * sizeof(uint64_t);
        std::copy(bin.symbol_table.begin(), bin.symbol_table.end(), &ret.data()[symbol_table_offset]);
        dvle->symbol_table_offset = symbol_table_offset - dvle_offset;
        dvle->symbol_table_size = bin.symbol_table.size();

        ret.resize(symbol_table_offset + bin.symbol_table.size());

        return ret;
    }
};

} // namespace
