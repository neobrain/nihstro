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

#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>
#include <stdint.h>
#include "bit_field.h"

#include "shader_binary.h"
#include "shader_bytecode.h"

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

    std::fstream file(argv[1], std::fstream::in | std::fstream::binary);

    DVLBHeader dvlb_header;
    file.read((char*)&dvlb_header, sizeof(dvlb_header));
    if (dvlb_header.magic_word != DVLBHeader::MAGIC_WORD) {
        std::cout << "Wrong DVLB magic word: Got " << std::hex << dvlb_header.magic_word << std::endl;
        return 0;
    }

    std::vector<uint32_t> dvle_offsets;
    std::vector<DVLEHeader> dvle_headers;
    dvle_offsets.resize(dvlb_header.num_programs);
    dvle_headers.resize(dvlb_header.num_programs);
    for (auto& offset : dvle_offsets) {
        file.read((char*)&offset, sizeof(offset));
    }
    uint32_t dvlp_offset = file.tellg(); // DVLP comes directly after the DVLE offset table

    for (int i = 0; i < dvlb_header.num_programs; ++i) {
        auto& dvle_header = dvle_headers[i];
        file.seekg(dvle_offsets[i]);
        file.read((char*)&dvle_header, sizeof(dvle_header));
        if (dvle_header.magic_word != DVLEHeader::MAGIC_WORD) {
            std::cout << "Wrong DVLE header in DVLE #" << i << ": " << std::hex << dvle_header.magic_word << std::endl;
        }
    }

    DVLPHeader dvlp_header;
    file.seekg(dvlp_offset);
    file.read((char*)&dvlp_header, sizeof(dvlp_header));
    if (dvlp_header.magic_word != DVLPHeader::MAGIC_WORD) {
        std::cout << "Wrong DVLP magic word: Got " << std::hex << dvlp_header.magic_word << std::endl;
        return 0;
    }

    // TODO: Size restriction
    SwizzlePattern swizzle_patterns[1024];
    file.seekg(dvlp_offset + dvlp_header.unk1_offset);
    file.read((char*)swizzle_patterns, 8 * dvlp_header.unk1_num_entries);

    auto ReadSymbol = [](std::fstream& file, uint32_t offset) {
                          // TODO: It's annoying that we just have to blindly read as many characters as possible...
                          //       Maybe there's a better way to do this?
                          char buffer[200];
                          file.seekg(offset);
                          uint32_t num_read = file.readsome(buffer, sizeof(buffer));
                          buffer[num_read - 1] = '\0';
                          return std::string(buffer);
                      };

    // TODO: Not sure how filenames should actually be parsed...
    //       currently we just jump to the filename symbol table and read whatever is there.
    //       Is a filename count specified anywhere?
    std::vector<std::string> filenames;
    filenames.resize(dvlb_header.num_programs);
    uint32_t offset = dvlp_offset + dvlp_header.filename_symbol_offset;
    for (int i = 0; i < dvlb_header.num_programs; ++i) {
        auto& filename = filenames[i];
        filename = ReadSymbol(file, offset);
        offset += filename.length() + 1;
        std::cout << "DVLE " << std::setw(3) << std::dec << i
                  << ": offset: 0x" << std::hex << std::setfill('0') << std::setw(4) << offset << ", "
                  << "byte offset to main: 0x" << std::hex << std::setw(8) << 4 * dvle_headers[i].main_offset_words << "\", "
                  << "original filename \"" << filenames[i]
                  << std::setfill(' ') << std::dec << std::endl;
    }

    std::cout << "Got " << dvlb_header.num_programs << " DVLE headers" << std::endl;

    if (argc < 3) {
        std::cout << "Error: No DVLE index given" << std::endl;
        return 0;
    }

    uint32_t dvle_index = std::stoi(std::string(argv[2]));

    if (dvle_index >= dvlb_header.num_programs) {
        std::cout << "Error: Invalid DVLE index " << dvle_index << "given" << std::endl;
        return 0;
    }

    auto& dvle_header = dvle_headers[dvle_index];
    auto& dvle_offset = dvle_offsets[dvle_index];

    uint32_t symbol_table_offset = dvle_offset + dvle_header.symbol_table_offset;

    uint32_t constant_table_offset = dvle_offset + dvle_header.constant_table_offset;
    struct ConstantInfo {
        union {
            BitField<0, 1, uint32_t> is_float32; // Custom extension, NOT OFFICIALLY SUPPORTED, only added to support loading citra's shader dumps until citra can convert floats to float24
            BitField<16, 8, uint32_t> regid;
            uint32_t full_first_word;
        };

        // float24 values..
        uint32_t x;
        uint32_t y;
        uint32_t z;
        uint32_t w;
    };
    std::vector<ConstantInfo> constant_table;
    constant_table.resize(dvle_header.constant_table_size);

    file.seekg(constant_table_offset);
    for (int i = 0; i < dvle_header.constant_table_size; ++i) {
        auto& info = constant_table[i];
        file.read((char*)&info, sizeof(ConstantInfo));
        if (info.is_float32) {
            std::cout << "Constant register info:  const" << info.regid.Value()
                      << " = (" << *(float*)&info.x << ", " << *(float*)&info.y
                      << ", " << *(float*)&info.z << ", " << *(float*)&info.w << ")"
                      << "  (raw: 0x" << std::hex << std::setfill('0') << std::setw(8) << info.full_first_word
                      << " 0x" << std::setw(8) << info.x << " 0x" << std::setw(8) << info.y
                      << " 0x" << std::setw(8) << info.z << " 0x" << std::setw(8) << info.w << std::dec << std::setfill( ' ') << ")"
                      << std::endl;
        } else {
            std::cout << "Constant register info:  const" << info.regid.Value()
                      << " = (" << float24::FromRawFloat24(info.x).ToFloat32() << ", " << float24::FromRawFloat24(info.y).ToFloat32()
                      << ", " << float24::FromRawFloat24(info.z).ToFloat32() << ", " << float24::FromRawFloat24(info.w).ToFloat32() << ")"
                      << "  (raw: 0x" << std::hex << std::setfill('0') << std::setw(8) << info.full_first_word
                      << " 0x" << std::setw(8) << info.x << " 0x" << std::setw(8) << info.y
                      << " 0x" << std::setw(8) << info.z << " 0x" << std::setw(8) << info.w << std::dec << std::setfill( ' ') << ")"
                      << std::endl;
        }
    }

    uint32_t label_table_offset = dvle_offset + dvle_header.label_table_offset;
    struct LabelTableEntry{
        BitField<0, 8, uint32_t> id;
        uint32_t program_offset;
        uint32_t unk;
        uint32_t name_offset;
    };
    std::vector<LabelTableEntry> label_table;
    std::map<uint32_t, std::string> labels;
    label_table.resize(dvle_header.label_table_size);

    file.seekg(label_table_offset);
    for (int i = 0; i < dvle_header.label_table_size; ++i) {
        file.read((char*)&label_table[i], sizeof(LabelTableEntry));
    }

    for (const auto& label_info : label_table) {
        labels.insert({label_info.program_offset, ReadSymbol(file, symbol_table_offset + label_info.name_offset)});
        std::cout << "Found label \"" << labels[label_info.program_offset] << "\" at program offset 0x" << std::hex << 4 * label_info.program_offset << std::endl;
    }

    // output register information
    union OutputRegisterInfo {
        enum Type : uint64_t {
            POSITION  = 0,
            COLOR     = 2,
            TEXCOORD0 = 3,
            TEXCOORD1 = 5,
            TEXCOORD2 = 6,
        };

        BitField< 0, 64, uint64_t> hex;

        BitField< 0, 16, Type> type;
        BitField<16, 16, uint64_t> id;
        BitField<32,  4, uint64_t> component_mask;
        BitField<32, 32, uint64_t> descriptor;

        std::string GetMask() const {
            std::string ret;
            if (component_mask & 1) ret += "x";
            if (component_mask & 2) ret += "y";
            if (component_mask & 4) ret += "z";
            if (component_mask & 8) ret += "w";
            return ret;
        }

        std::string GetPlainName() const {
            std::map<Type, std::string> map = {
                { POSITION,  "out.pos"},
                { COLOR,     "out.col"},
                { TEXCOORD0, "out.tex0"},
                { TEXCOORD1, "out.tex1"},
                { TEXCOORD2, "out.tex2"},
            };
            auto it = map.find(type);
            if (it != map.end())
                return it->second;
            else
                return "out.unk";
        }

        std::string GetFullName() const {
            return GetPlainName() + "." + GetMask();
        }
    };
    std::vector<OutputRegisterInfo> output_register_info;
    output_register_info.resize(dvle_header.output_register_table_size);
    file.seekg(dvle_offset + dvle_header.output_register_table_offset);
    for (auto& info : output_register_info) {
        file.read((char*)&info, sizeof(OutputRegisterInfo));
        std::cout << "Output register info:  o" << info.id.Value() << " = " << std::setw(13) << info.GetFullName() << " (" << std::hex << std::setw(16) << std::setfill('0') << (uint64_t)info.hex << std::setfill(' ') << ")" << std::endl;
    }

    struct UniformInfo {
        struct {
            uint32_t symbol_offset;
            union {
                BitField< 0, 16, uint32_t> reg_start;
                BitField<16, 16, uint32_t> reg_end; // inclusive
            };
        } basic;
        std::string name;
    };
    std::vector<UniformInfo> uniform_table;
    uniform_table.resize(dvle_header.uniform_table_size);

    uint32_t uniform_table_offset = dvle_offset + dvle_header.uniform_table_offset;
    file.seekg(uniform_table_offset);
    for (int i = 0; i < dvle_header.uniform_table_size; ++i) {
        file.read((char*)&uniform_table[i].basic, sizeof(uniform_table[i].basic));
    }

    for (auto& uniform_info : uniform_table) {
        uniform_info.name = ReadSymbol(file, symbol_table_offset + uniform_info.basic.symbol_offset);
        std::cout << "Found uniform symbol \"" << std::setw(20) << uniform_info.name << "\" for registers 0x" << std::setfill('0') << std::setw(2) << uniform_info.basic.reg_start << "-0x" << std::setw(2) << uniform_info.basic.reg_end << " at offset 0x" << std::hex << symbol_table_offset + uniform_info.basic.symbol_offset << std::setfill(' ') << std::endl;
    }


    uint32_t main_offset = dvlp_offset + dvlp_header.binary_offset;
    std::cout << "Disassembling " << dvlp_header.binary_size_words << " bytes from offset "
              << dvlp_offset << " + " << dvlp_header.binary_offset << " = " << main_offset << " (main at byte offset " << "0x" << std::hex << 4 * dvle_header.main_offset_words << ")" << std::endl;
    file.seekg(main_offset);

    for (uint32_t word = 0; word < dvlp_header.binary_size_words; ++word) {
        auto HasLabel = [&labels](uint32_t offset) {
                            return labels.find(offset) != labels.end();
                        };
        auto GetLabel = [&labels](uint32_t offset) -> std::string {
                            auto it = labels.find(offset);
                            if (it != labels.end())
                                return it->second;
                            return "";
                        };

        std::cout.flags(std::ios::left | std::ios::hex);
        if (HasLabel(word)) {
            std::cout << std::setw(8) << std::right << std::setfill('0') << 4*word
                      << " [--------] " << GetLabel(word) << ":" << std::endl;
        }

        Instruction instr;
        file.seekg(main_offset + 4 * word);
        file.read((char*)&instr, sizeof(instr));

        std::cout << std::setw(8) << std::right << std::setfill('0') << 4*word << " "
                  << "[" << std::setw(8) << std::right << std::setfill('0') << instr.hex << "]     "
                  << std::setw(7) << std::left << std::setfill(' ') << instr.GetOpCodeName();

        const SwizzlePattern& swizzle = swizzle_patterns[2*instr.common.operand_desc_id];

        // TODO: Not sure if name lookup works properly, yet!
        auto GetDestName = [&](const decltype(instr.common.dest) dest, const SwizzlePattern& swizzle) -> std::string {
                               if (dest >= 0x10) {
                                   for (const auto& uniform_info : uniform_table) {
                                       // TODO: Is there any point in testing for this? Might it describe temporaries?
                                       if (dest >= uniform_info.basic.reg_start &&
                                           dest <= uniform_info.basic.reg_end) {
                                           return uniform_info.name;
                                       }
                                   }
                               } else if (dest < 0x8) {
                                   // TODO: This one still needs some prettification in case
                                   //       multiple output_infos describing this output register
                                   //       are found.
                                   std::string ret;
                                   for (const auto& output_info : output_register_info) {
                                       if (dest != output_info.id)
                                           continue;

                                       if (0 == (swizzle.dest_mask & output_info.component_mask))
                                           continue;

                                       // Add a vertical bar so that we have at least *some*
                                       // indication that we hit multiple matches.
                                       if (!ret.empty())
                                           ret += "|";

                                       ret += output_info.GetFullName();
                                   }
                                   if (!ret.empty())
                                       return ret;
                               }
                               return "(?)";
                           };
        auto GetSrc1Name = [&](const decltype(instr.common.src1) src1) -> std::string {
                               for (const auto& uniform_info : uniform_table) {
                                   if (src1 >= uniform_info.basic.reg_start &&
                                       src1 <= uniform_info.basic.reg_end) {
                                       return uniform_info.name;
                                   }
                               }
                               // Constants and uniforms really are the same internally
                               for (const auto& constant_info : constant_table) {
                                   if (src1 - 0x20 == constant_info.regid) {
                                       return "const" + std::to_string(constant_info.regid.Value());
                                   }
                               }
                               return "(?)";
                           };
        auto GetSrc2Name = [&](const decltype(instr.common.src2) src2) -> std::string {
                               for (const auto& uniform_info : uniform_table) {
                                   if (src2 >= uniform_info.basic.reg_start &&
                                       src2 <= uniform_info.basic.reg_end) {
                                       return uniform_info.name;
                                   }
                               }
                               return "(?)";
                           };

        switch (instr.opcode) {
        // common, uses DEST and SRC1:
        case Instruction::OpCode::RCP:
        case Instruction::OpCode::RSQ:
        case Instruction::OpCode::MOV:
            std::cout << std::setw(4) << std::right << instr.common.dest.GetRegisterName() << "." << swizzle.DestMaskToString() << "  "
                      << std::setw(4) << std::right << ((swizzle.negate ? "-" : " ") + instr.common.src1.GetRegisterName()) << "." << swizzle.SelectorToString(false) << "   "
                      << "           " << instr.common.operand_desc_id.Value() << " flag:" << instr.common.unk2.Value()
                      << ";      " << GetDestName(instr.common.dest, swizzle) << ",  " << GetSrc1Name(instr.common.src1) << std::endl;
            break;

        // common, uses DEST, SRC1 and SRC2:
        case Instruction::OpCode::ADD:
        case Instruction::OpCode::DP3:
        case Instruction::OpCode::DP4:
        case Instruction::OpCode::MUL:
        case Instruction::OpCode::MAX:
        case Instruction::OpCode::MIN:
            std::cout << std::setw(4) << std::right << instr.common.dest.GetRegisterName() << "." << swizzle.DestMaskToString() << "  "
                      << std::setw(4) << std::right << ((swizzle.negate ? "-" : "") + instr.common.src1.GetRegisterName()) << "." << swizzle.SelectorToString(false) << "  "
                      << std::setw(4) << std::right << instr.common.src2.GetRegisterName() << "." << swizzle.SelectorToString(true) << "   "
                      << instr.common.operand_desc_id.Value() << " flag:" << instr.common.unk2.Value()
                      << ";      " << GetDestName(instr.common.dest, swizzle) << " <- " << GetSrc1Name(instr.common.src1) << ", " << GetSrc2Name(instr.common.src2) << std::endl;
            break;

        case Instruction::OpCode::CALL:
            std::cout << "to 0x" << std::setw(4) << std::right << std::setfill('0') << 4 * instr.flow_control.offset_words << std::setfill(' ') << "  ("
                      << std::setw(4) << std::right << (int)instr.flow_control.num_instructions << " words)    "
                      << GetLabel(instr.flow_control.offset_words) << std::endl;
            break;

        default:
            std::cout << std::endl;
            break;
        }
    }

    std::cout << std::endl << "Swizzle patterns:" << std::endl;

    for (int i = 0; i < dvlp_header.unk1_num_entries; ++i) {
        const auto& pattern = swizzle_patterns[2*i];
        std::cout << "(" << std::setw(3) << std::right << std::hex << i << ") " << std::setw(8) << pattern.hex << ": " << pattern.dest_mask.Value() << "   " <<
                     " " << ((pattern.hex>>4)&1) << "   " <<
                     " " << (int)pattern.src1_selector_3.Value() << " " << (int)pattern.src1_selector_2.Value() <<
                     " " << (int)pattern.src1_selector_1.Value() << " " << (int)pattern.src1_selector_0.Value() << "   " <<
                     " " << ((pattern.hex>>13)&1) << "   " <<
                     " " << (int)pattern.src2_selector_3.Value() << " " << (int)pattern.src2_selector_2.Value() <<
                     " " << (int)pattern.src2_selector_1.Value() << " " << (int)pattern.src2_selector_0.Value() << "   " <<
                     " " << std::setw(3) << ((pattern.hex>>22)&0x1FF) << "   " <<
                     " " << (int)pattern.flag.Value() << "    " << std::setw(8) << std::setfill('0') << swizzle_patterns[2*i+1].hex << std::setfill(' ') << std::endl;
    }

    return 0;
}
