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
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>
#include <stdint.h>
#include "bit_field.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#pragma pack(1)
struct DVLBHeader {
    enum : u32 {
        MAGIC_WORD = 0x424C5644, // "DVLB"
    };

    u32 magic_word;
    u32 num_programs;
//    u32 dvle_offset_table[];
};
static_assert(sizeof(DVLBHeader) == 0x8, "Incorrect structure size");

struct DVLPHeader {
    enum : u32 {
        MAGIC_WORD = 0x504C5644, // "DVLP"
    };

    u32 magic_word;
    u32 version;
    u32 binary_offset;  // relative to DVLP start
    u32 binary_size_words;
    u32 unk1_offset;
    u32 unk1_num_entries;
    u32 filename_symbol_offset;
};
static_assert(sizeof(DVLPHeader) == 0x1C, "Incorrect structure size");

struct DVLEHeader {
    enum : u32 {
        MAGIC_WORD = 0x454c5644, // "DVLE"
    };

    enum class ShaderType : u8 {
        VERTEX = 0,
        GEOMETRY = 1,
    };

    u32 magic_word;
    u16 pad1;
    ShaderType type;
    u8 pad2;
    u32 main_offset_words; // offset within binary blob
    u32 endmain_offset_words;
    u32 pad3;
    u32 pad4;
    u32 constant_table_offset;
    u32 constant_table_size; // number of entries
    u32 label_table_offset;
    u32 label_table_size;
    u32 output_register_table_offset;
    u32 output_register_table_size;
    u32 uniform_table_offset;
    u32 uniform_table_size;
    u32 symbol_table_offset;
    u32 symbol_table_size;

};
static_assert(sizeof(DVLEHeader) == 0x40, "Incorrect structure size");


/*enum class Instructions : u32
{
};*/

union Instruction {
    enum class OpCode : u32 {
        ADD =  0x0,
        DP3 =  0x1,
        DP4 =  0x2,

        MUL =  0x8,

        MAX =  0xC,
        MIN =  0xD,
        RCP =  0xE,
        RSQ =  0xF,

        MOV = 0x13,

        RET = 0x21,
        FLUSH = 0x22,


        CALL = 0x24,
        CALL2 = 0x25,
        CALLC = 0x26,
        IFU = 0x27,
        CMP = 0x2E,
    };

    std::string GetOpCodeName() const {
        std::map<OpCode, std::string> map = {
            { OpCode::ADD, "ADD" },
            { OpCode::DP3, "DP3" },
            { OpCode::DP4, "DP4" },
            { OpCode::MUL, "MUL" },
            { OpCode::MAX, "MAX" },
            { OpCode::MIN, "MIN" },
            { OpCode::RCP, "RCP" },
            { OpCode::RSQ, "RSQ" },
            { OpCode::MOV, "MOV" },
            { OpCode::RET, "RET" },
            { OpCode::FLUSH, "FLS" },
            { OpCode::CALL, "CALL" },
            { OpCode::CMP, "CMP" },
        };
        auto it = map.find(opcode);
        if (it == map.end())
            return std::string("UNK") + std::to_string(static_cast<int>(opcode.Value()));
        else
            return it->second;
    }

    u32 hex;

    BitField<0x1a, 6, OpCode> opcode;

    // General notes:
    //
    // When two input registers are used, one of them uses a 5-bit index while the other
    // one uses a 7-bit index. This is because at most one floating point uniform may be used
    // as an input.


    // Format used e.g. by arithmetic instructions and comparisons
    // "src1" and "src2" specify register indices (i.e. indices referring to groups of 4 floats),
    // while "dest" addresses individual floats.
    union {
        BitField<0x00, 0x7, u32> operand_desc_id;

        struct : BitField<0x07, 0x5, u32>
        {
            enum RegisterType {
                Input,
                Temporary,
                Unknown
            };

            RegisterType GetRegisterType() const {
                if (Value() < 0x10)
                    return Input;
                else if (Value() < 0x20)
                    return Temporary;
                else
                    return Unknown;
            }

            int GetIndex() const {
                if (GetRegisterType() == Input)
                    return Value();
                else if (GetRegisterType() == Temporary)
                    return Value() - 0x10;
                else
                    return Value();
            }

            std::string GetRegisterName() const {
                std::map<RegisterType, std::string> type = {
                    { Input, "i" },
                    { Temporary, "t" },
                    { Unknown, "u" }
                };
                return type[GetRegisterType()] + std::to_string(GetIndex());
            }
        } src2;

        struct : BitField<0x0c, 0x7, u32>
        {
            enum RegisterType {
                Input,
                Temporary,
                FloatUniform,
                Unknown
            };

            RegisterType GetRegisterType() const {
                if (Value() < 0x10)
                    return Input;
                else if (Value() < 0x20)
                    return Temporary;
                else if (Value() < 0x80)
                    return FloatUniform;
                else
                    return Unknown;
            }

            int GetIndex() const {
                if (GetRegisterType() == Input)
                    return Value();
                else if (GetRegisterType() == Temporary)
                    return Value() - 0x10;
                else if (GetRegisterType() == FloatUniform)
                    return Value() - 0x20;
                else
                    return Value();
            }

            std::string GetRegisterName() const {
                std::map<RegisterType, std::string> type = {
                    { Input, "i" },
                    { Temporary, "t" },
                    { FloatUniform, "f" },
                    { Unknown, "u" }
                };
                return type[GetRegisterType()] + std::to_string(GetIndex());
            }
        } src1;

        BitField<0x13, 0x2, u32> unk2; // 3dbrew calls this FLAG

        struct : BitField<0x15, 0x5, u32>
        {
            enum RegisterType {
                Output,
                Temporary,
                Unknown
            };

            RegisterType GetRegisterType() const {
                if (Value() < 0x8)
                    return Output;
                else if (Value() < 0x10)
                    return Unknown;
                else if (Value() < 0x20)
                    return Temporary;
                else
                    return Unknown;
            }

            int GetIndex() const {
                if (GetRegisterType() == Output)
                    return Value();
                else if (GetRegisterType() == Temporary)
                    return Value() - 0x10;
                else
                    return Value();
            }

            std::string GetRegisterName() const {
                std::map<RegisterType, std::string> type = {
                    { Output, "o" },
                    { Temporary, "t" },
                    { Unknown, "u" }
                };
                return type[GetRegisterType()] + std::to_string(GetIndex());
            }
        } dest;
    } common;

    // Format used for flow control instructions ("if")
    union {
        BitField<0x00, 0x8, u32> num_instructions;
        BitField<0x0a, 0xc, u32> offset_words;
    } flow_control;
};
static_assert(sizeof(Instruction) == 0x4, "Incorrect structure size");
static_assert(std::is_standard_layout<Instruction>::value, "Structure does not have standard layout");

union SwizzlePattern {
    u32 hex;

    enum class Selector : u32 {
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

    bool DestComponentEnabled(int i) const {
        return (dest_mask & (0x8 >> i));
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
    BitField< 0, 4, u32> dest_mask;

    BitField< 4, 1, u32> negate; // negates src1

    BitField< 5, 2, Selector> src1_selector_3;
    BitField< 7, 2, Selector> src1_selector_2;
    BitField< 9, 2, Selector> src1_selector_1;
    BitField<11, 2, Selector> src1_selector_0;

    BitField<14, 2, Selector> src2_selector_3;
    BitField<16, 2, Selector> src2_selector_2;
    BitField<18, 2, Selector> src2_selector_1;
    BitField<20, 2, Selector> src2_selector_0;

    BitField<31, 1, u32> flag; // not sure what this means, maybe it's the sign?
};
static_assert(sizeof(SwizzlePattern) == 0x4, "Incorrect structure size");


#pragma pack()

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

    std::vector<u32> dvle_offsets;
    std::vector<DVLEHeader> dvle_headers;
    dvle_offsets.resize(dvlb_header.num_programs);
    dvle_headers.resize(dvlb_header.num_programs);
    for (auto& offset : dvle_offsets) {
        file.read((char*)&offset, sizeof(offset));
    }
    u32 dvlp_offset = file.tellg(); // DVLP comes directly after the DVLE offset table

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

    std::cout << "Got " << dvlb_header.num_programs << " DVLE headers" << std::endl;

    auto ReadSymbol = [](std::fstream& file, u32 offset) {
                          // TODO: It's annoying that we just have to blindly read as many characters as possible...
                          //       Maybe there's a better way to do this?
                          char buffer[200];
                          file.seekg(offset);
                          u32 num_read = file.readsome(buffer, sizeof(buffer));
                          buffer[num_read - 1] = '\0';
                          return std::string(buffer);
                      };

    // TODO: Not sure how filenames should actually be parsed...
    //       currently we just jump to the filename symbol table and read whatever is there.
    //       Is a filename count specified anywhere?
    std::vector<std::string> filenames;
    filenames.resize(dvlb_header.num_programs);
    u32 offset = dvlp_offset + dvlp_header.filename_symbol_offset;
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

    if (argc < 3) {
        std::cout << "Error: No DVLE index given" << std::endl;
        return 0;
    }

    u32 dvle_index = std::stoi(std::string(argv[2]));

    if (dvle_index >= dvlb_header.num_programs) {
        std::cout << "Error: Invalid DVLE index " << dvle_index << "given" << std::endl;
        return 0;
    }

    // TODO: Size restriction
    SwizzlePattern swizzle_patterns[1024];
    file.seekg(dvlp_offset + dvlp_header.unk1_offset);
    std::cout << "Reading " << dvlp_header.unk1_num_entries << " swizzle pattern entries from offset 0x" << std::hex << dvlp_header.unk1_offset << std::endl;
    file.read((char*)swizzle_patterns, 8 * dvlp_header.unk1_num_entries);

    auto& dvle_header = dvle_headers[dvle_index];
    auto& dvle_offset = dvle_offsets[dvle_index];

    u32 symbol_table_offset = dvle_offset + dvle_header.symbol_table_offset;

    u32 label_table_offset = dvle_offset + dvle_header.label_table_offset;
    struct LabelTableEntry{
        BitField<0, 8, u32> id;
        u32 program_offset;
        u32 unk;
        u32 name_offset;
    };
    std::vector<LabelTableEntry> label_table;
    std::map<u32, std::string> labels;
    label_table.resize(dvle_header.label_table_size);

    file.seekg(label_table_offset);
    for (int i = 0; i < dvle_header.label_table_size; ++i) {
        file.read((char*)&label_table[i], sizeof(LabelTableEntry));
    }

    for (const auto& label_info : label_table) {
        labels.insert({label_info.program_offset, ReadSymbol(file, symbol_table_offset + label_info.name_offset)});
        std::cout << "Found label \"" << labels[label_info.program_offset] << "\" at program offset 0x" << std::hex << 4 * label_info.program_offset << std::endl;
    }

    struct UniformInfo {
        struct {
            u32 symbol_offset;
            union {
                BitField< 0, 16, u32> reg_start;
                BitField<16, 16, u32> reg_end; // inclusive
            };
        } basic;
        std::string name;
    };
    std::vector<UniformInfo> uniform_table;
    uniform_table.resize(dvle_header.uniform_table_size);

    u32 uniform_table_offset = dvle_offset + dvle_header.uniform_table_offset;
    file.seekg(uniform_table_offset);
    for (int i = 0; i < dvle_header.uniform_table_size; ++i) {
        file.read((char*)&uniform_table[i].basic, sizeof(uniform_table[i].basic));
    }

    for (auto& uniform_info : uniform_table) {
        uniform_info.name = ReadSymbol(file, symbol_table_offset + uniform_info.basic.symbol_offset);
        std::cout << "Found uniform symbol \"" << std::setw(20) << uniform_info.name << "\" for registers 0x" << std::setfill('0') << std::setw(2) << uniform_info.basic.reg_start << "-0x" << std::setw(2) << uniform_info.basic.reg_end << " at offset 0x" << std::hex << symbol_table_offset + uniform_info.basic.symbol_offset << std::setfill(' ') << std::endl;
    }


    u32 main_offset = dvlp_offset + dvlp_header.binary_offset;
    std::cout << "Disassembling " << dvlp_header.binary_size_words << " bytes from offset "
              << dvlp_offset << " + " << dvlp_header.binary_offset << " = " << main_offset << " (main at byte offset " << "0x" << std::hex << 4 * dvle_header.main_offset_words << ")" << std::endl;
    file.seekg(main_offset);

    for (u32 word = 0; word < dvlp_header.binary_size_words; ++word) {
        auto HasLabel = [&labels](u32 offset) {
                            return labels.find(offset) != labels.end();
                        };
        auto GetLabel = [&labels](u32 offset) -> std::string {
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
        auto GetDestName = [&uniform_table](const decltype(instr.common.dest) dest) -> std::string {
                               for (const auto& uniform_info : uniform_table) {
                                   if (dest >= uniform_info.basic.reg_start &&
                                       dest <= uniform_info.basic.reg_end &&
                                       dest >= 0x10) {
                                       return uniform_info.name;
                                   }
                               }
                               return "(?)";
                           };
        auto GetSrc1Name = [&uniform_table](const decltype(instr.common.src1) src1) -> std::string {
                               for (const auto& uniform_info : uniform_table) {
                                   if (src1 >= uniform_info.basic.reg_start &&
                                       src1 <= uniform_info.basic.reg_end) {
                                       return uniform_info.name;
                                   }
                               }
                               return "(?)";
                           };
        auto GetSrc2Name = [&uniform_table](const decltype(instr.common.src2) src2) -> std::string {
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
                      << "           " << instr.common.operand_desc_id.Value() << " unk2:" << instr.common.unk2.Value()
                      << ";      " << GetDestName(instr.common.dest) << ",  " << GetSrc1Name(instr.common.src1) << std::endl;
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
                      << ";      " << GetDestName(instr.common.dest) << " <- " << GetSrc1Name(instr.common.src1) << ", " << GetSrc2Name(instr.common.src2) << std::endl;
            break;

        case Instruction::OpCode::CALL:
            std::cout << std::setw(4) << std::right << (int)instr.flow_control.offset_words << "  "
                      << std::setw(4) << std::right << (int)instr.flow_control.num_instructions << " words    "
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
