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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <stdint.h>

#include "nihstro/bit_field.h"
#include "nihstro/shader_bytecode.h"
#include "nihstro/parser_shbin.h"

using namespace nihstro;

struct float24 {
    static float24 FromFloat32(float val) {
        float24 ret;
        ret.value = val;
        return ret;
    }

    // 16 bit mantissa, 7 bit exponent, 1 bit sign
    // TODO: No idea if this works as intended
    static float24 FromRawFloat24(uint32_t hex) {
        float24 ret;
        if ((hex & 0xFFFFFF) == 0) {
            ret.value = 0;
        } else {
            uint32_t mantissa = hex & 0xFFFF;
            uint32_t exponent = (hex >> 16) & 0x7F;
            uint32_t sign = hex >> 23;
            ret.value = std::pow(2.0f, (float)exponent-63.0f) * (1.0f + mantissa * std::pow(2.0f, -16.f));
            if (sign)
                ret.value = -ret.value;
        }
        return ret;
    }

    // Not recommended for anything but logging
    float ToFloat32() const {
        return value;
    }

private:
    // Stored as a regular float, merely for convenience
    // TODO: Perform proper arithmetic on this!
    float value;
};

int main(int argc, char *argv[])
{
    // TODO: Make this check portable!
    if (argc < 2) {
        std::cout << "Error: No filename given" << std::endl;
        return 0;
    }

    ShbinParser parser;
    try {
        parser.ReadHeaders(argv[1]);

        for (int i = 0; i < parser.GetDVLBHeader().num_programs; ++i) {
            auto& filename = parser.GetFilename(i);
            std::cout << "DVLE " << std::setw(3) << std::dec << i << ": "
//                      << "offset: 0x" << std::hex << std::setfill('0') << std::setw(4) << offset << ", "
                      << std::hex << std::setfill('0')
                      << "byte offset to main: 0x" << std::hex << std::setw(8) << 4 * parser.GetDVLEHeader(i).main_offset_words << "\", "
                      << "original filename \"" << filename << "\""
                      << std::setfill(' ') << std::dec << std::endl;
        }

        std::cout << "Got " << parser.GetDVLBHeader().num_programs << " DVLE headers" << std::endl;

        if (argc < 3) {
            std::cout << "Error: No DVLE index given" << std::endl;
            return 0;
        }

        uint32_t dvle_index = std::stoi(std::string(argv[2]));
        parser.ReadDVLE(dvle_index);

        auto& dvle_header = parser.dvle_headers[dvle_index];
        for (int i = 0; i < parser.shader_info.constant_table.size(); ++i) {
            auto& info = parser.shader_info.constant_table[i];

            switch (info.type) {
            case ConstantInfo::Float:
                std::cout << "Constant register info:  " << GetRegisterName(RegisterType::FloatUniform) << info.regid.Value()
                          << " = (" << float24::FromRawFloat24(info.f.x).ToFloat32() << ", " << float24::FromRawFloat24(info.f.y).ToFloat32()
                          << ", " << float24::FromRawFloat24(info.f.z).ToFloat32() << ", " << float24::FromRawFloat24(info.f.w).ToFloat32() << ")"
                          << "  (raw: 0x" << std::hex << std::setfill('0') << std::setw(8) << info.full_first_word
                          << " 0x" << std::setw(8) << info.f.x << " 0x" << std::setw(8) << info.f.y
                          << " 0x" << std::setw(8) << info.f.z << " 0x" << std::setw(8) << info.f.w << std::dec << std::setfill( ' ') << ")"
                          << std::endl;
                break;

            case ConstantInfo::Int:
                std::cout << "Constant register info:  " << GetRegisterName(RegisterType::IntUniform) << info.regid.Value()
                          << " = (" << (int)info.i.x << ", " << (int)info.i.y
                          << ", " << (int)info.i.z << ", " << (int)info.i.w << ")"
                          << "  (raw: 0x" << std::hex << std::setfill('0') << std::setw(8) << info.full_first_word
                          << std::dec << std::setfill( ' ') << ")"
                          << std::endl;
                break;

            case ConstantInfo::Bool:
                std::cout << "Constant register info:  " << GetRegisterName(RegisterType::BoolUniform) << info.regid.Value()
                          << " = " << std::boolalpha << (bool)info.b
                          << "  (raw: 0x" << std::hex << std::setfill('0') << std::setw(8) << info.full_first_word
                          << std::dec << std::setfill( ' ') << ")"
                          << std::endl;
                break;

            default:
            {
                std::stringstream str("Unknown constant type: ");
                str << std::hex << info.type.Value();
                throw str.str();
            }
            }
        }

        for (int i = 0; i < parser.shader_info.label_table.size(); ++i) {
            const auto& label_info = parser.shader_info.label_table[i];
            std::cout << "Found label \"" << parser.shader_info.labels[label_info.program_offset]
                      << "\" at program offset 0x" << std::hex << 4 * label_info.program_offset
                      << std::endl;
        }

        for (auto& info : parser.shader_info.output_register_info)
            std::cout << "Output register info:  " << GetRegisterName(RegisterType::Output) << info.id.Value() << '.' << std::setw(4) << std::left << info.GetMask()
                      << " as " << std::setw(8) << info.GetSemanticName()
                      << " (" << std::hex << std::setw(16) << std::setfill('0') << (uint64_t)info.hex << std::setfill(' ') << ")" << std::endl;

        if (!parser.shader_info.uniform_table.empty()) {
            size_t max_uniform_name_length = std::max_element(parser.shader_info.uniform_table.begin(), parser.shader_info.uniform_table.end(),
                                                              [](const UniformInfo& i1, UniformInfo& i2) { return i1.name.length() < i2.name.length(); }
                                                             )->name.length();
            for (auto& uniform_info : parser.shader_info.uniform_table) {
                bool is_range = (uniform_info.basic.reg_start != uniform_info.basic.reg_end);

                std::cout << "Found uniform symbol \"" << std::setw(max_uniform_name_length) << uniform_info.name
                          << "\" for register" << (is_range ? "s " : "  ") << std::dec
                          << GetRegisterName(uniform_info.basic.GetStartType()) << uniform_info.basic.GetStartIndex();
                if (is_range)
                    std::cout << "-" << GetRegisterName(uniform_info.basic.GetEndType()) << uniform_info.basic.GetEndIndex();

                std::cout << std::endl;
            }
        }

// TODO:
//        std::cout << "Disassembling " << parser.GetDVLPHeader().binary_size_words << " bytes from offset "
//                  << dvlp_offset << " + " << dvlp_header.binary_offset << " = " << main_offset << " (main at byte offset " << "0x" << std::hex << 4 * dvle_header.main_offset_words << ")" << std::endl;
    } catch (const std::string& err) {
        std::cout << "Exception while reading \"" << argv[1] << "\": " << err << std::endl;
        return 1;
    } catch (const std::ios_base::failure& except) {
        std::cout << "Exception while reading \"" << argv[1] << "\": ios_base::failure \"" << except.what() << "\" (invalid shbin?)" << std::endl;
        return 1;
    } catch (const std::bad_alloc&) {
        std::cout << "Exception while reading \"" << argv[1] << "\": bad_alloc (invalid shbin?)" << std::endl;
        return 1;
    }

    const ShaderInfo& shader_info = parser.shader_info;
    for (uint32_t word = 0; word < shader_info.code.size(); ++word) {
        std::cout.flags(std::ios::left | std::ios::hex);
        if (shader_info.HasLabel(word)) {
            std::cout << std::setw(8) << std::right << std::setfill('0') << 4*word
                      << " [--------] " << shader_info.GetLabel(word) << ":" << std::endl;
        }

        Instruction instr = shader_info.code[word];
        OpCode opcode = instr.opcode.Value();

        std::cout << std::setw(8) << std::right << std::setfill('0') << 4*word << " "
                  << "[" << std::setw(8) << std::right << std::setfill('0') << instr.hex << "]     "
                  << std::setw(7) << std::left << std::setfill(' ') << opcode.GetInfo().name;

        const SwizzlePattern& swizzle = shader_info.swizzle_info[instr.common.operand_desc_id].pattern;

        // TODO: Not sure if name lookup works properly, yet!

        if (opcode.GetInfo().type == OpCode::Type::Arithmetic) {
            bool src_reversed = 0 != (opcode.GetInfo().subtype & OpCode::Info::SrcInversed);
            auto src1 = instr.common.GetSrc1(src_reversed);
            auto src2 = instr.common.GetSrc2(src_reversed);
            auto dest = instr.common.dest.Value();

            std::string src1_relative_address;
            if (!instr.common.AddressRegisterName().empty())
                src1_relative_address = "[" + instr.common.AddressRegisterName() + "]";

            if (opcode.GetInfo().subtype & OpCode::Info::Dest) {
                std::cout << std::setw(4) << std::right << dest.GetName() << "." << swizzle.DestMaskToString() << "  ";
            } else {
                std::cout << "    ";
            }

            if (opcode.GetInfo().subtype & OpCode::Info::Src1) {
                std::cout << std::setw(8) << std::right << ((swizzle.negate_src1 ? "-" : "") + src1.GetName()) + src1_relative_address << "." << swizzle.SelectorToString(false) << "  ";
            } else {
                std::cout << "           ";
            }

            if (opcode.GetInfo().subtype & OpCode::Info::CompareOps) {
                std::cout << instr.common.compare_op.ToString(instr.common.compare_op.x) << " " << instr.common.compare_op.ToString(instr.common.compare_op.y) << " ";
            } else {
            }

            if (opcode.GetInfo().subtype & OpCode::Info::Src2) {
                std::cout << std::setw(4) << std::right << (swizzle.negate_src2 ? "-" : "") + src2.GetName() << "." << swizzle.SelectorToString(true) << "   ";
            } else {
                std::cout << "            ";
            }

            std::cout << std::setw(2) << instr.common.operand_desc_id.Value() << " addr:" << instr.common.address_register_index.Value()
                      << ";      " << shader_info.LookupDestName(dest, swizzle) << " <- " << (swizzle.negate_src1 ? "-" : "") + shader_info.LookupSourceName(src1, instr.common.address_register_index);
            if (opcode.GetInfo().subtype & OpCode::Info::Src2)
                std::cout << ", " << (swizzle.negate_src2 ? "-" : "") + shader_info.LookupSourceName(src2, 0);

            std::cout << std::endl;
        } else if (opcode.GetInfo().type == OpCode::Type::Conditional) {
            std::cout << "if ";

            if (opcode.GetInfo().subtype & OpCode::Info::HasCondition) {
                const char* ops[] = {
                    " || ", " && ", "", ""
                };
                if (instr.flow_control.op != instr.flow_control.JustY)
                    std::cout << ((!instr.flow_control.refx) ? "!" : " ") << "cc.x";

                std::cout << ops[instr.flow_control.op];

                if (instr.flow_control.op != instr.flow_control.JustX)
                    std::cout << ((!instr.flow_control.refy) ? "!" : " ") << "cc.y";

                std::cout << " ";
            } else if (opcode.GetInfo().subtype & OpCode::Info::HasUniformIndex) {
                std::cout << "b" << instr.flow_control.bool_uniform_id << " ";
            }

            uint32_t target_addr = instr.flow_control.dest_offset;
            uint32_t target_addr_else = instr.flow_control.dest_offset;

            if (opcode.GetInfo().subtype & OpCode::Info::HasAlternative) {
                std::cout << "else jump to 0x" << std::setw(4) << std::right << std::setfill('0') << 4 * instr.flow_control.dest_offset
                          << " aka \"" << shader_info.GetLabel(instr.flow_control.dest_offset) << "\"";
            } else if (opcode.GetInfo().subtype & OpCode::Info::HasExplicitDest) {
                std::cout << "jump to 0x" << std::setw(4) << std::right << std::setfill('0') << 4 * instr.flow_control.dest_offset
                          << " aka \"" << shader_info.GetLabel(instr.flow_control.dest_offset) << "\"";
            } else {
                // TODO: Handle other cases
            }

            if (opcode.GetInfo().subtype & OpCode::Info::HasFinishPoint) {
                std::cout << "(return on " << std::setw(4) << std::right << std::setfill('0') << 4 * instr.flow_control.dest_offset + 4 * instr.flow_control.num_instructions << "\")";
            }

            std::cout << std::endl;
        } else {
            std::cout << std::endl;
        }
    }

    std::cout << std::endl << "Swizzle patterns:" << std::endl;

    for (int i = 0; i < shader_info.swizzle_info.size(); ++i) {
        const auto& info = shader_info.swizzle_info[i];
        const auto& pattern = info.pattern;
        std::cout << "(" << std::setw(3) << std::right << std::hex << i << ") " << std::setw(8) << std::setfill('0') << pattern.hex << ": " << pattern.dest_mask.Value() << "   " <<
                     " " << (int)pattern.negate_src1 << "  " <<
                     " " << (int)pattern.src1_selector_3.Value() << " " << (int)pattern.src1_selector_2.Value() <<
                     " " << (int)pattern.src1_selector_1.Value() << " " << (int)pattern.src1_selector_0.Value() << "   " <<
                     " " << (int)pattern.negate_src2 << "  " <<
                     " " << (int)pattern.src2_selector_3.Value() << " " << (int)pattern.src2_selector_2.Value() <<
                     " " << (int)pattern.src2_selector_1.Value() << " " << (int)pattern.src2_selector_0.Value() << "   " <<
                     " " << (int)pattern.negate_src3 << "  " <<
                     " " << (int)pattern.src3_selector_3.Value() << " " << (int)pattern.src3_selector_2.Value() <<
                     " " << (int)pattern.src3_selector_1.Value() << " " << (int)pattern.src3_selector_0.Value() << "   " <<
                     " " <<  std::setw(8) << std::setfill('0') << info.unknown << std::setfill(' ') << std::endl;
    }

    return 0;
}
