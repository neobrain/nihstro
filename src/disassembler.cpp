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
#include <sstream>
#include <vector>
#include <map>
#include <stdint.h>
#include "bit_field.h"

#include "nihstro/shader_binary.h"
#include "nihstro/shader_bytecode.h"

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

class ShbinParser {
public:
    void ReadHeaders(const std::string& filename) {
        file.exceptions(std::fstream::badbit | std::fstream::failbit | std::fstream::eofbit);
        file.open(filename, std::fstream::in | std::fstream::binary);

        file.seekg(0);
        file.read((char*)&dvlb_header, sizeof(dvlb_header));
        if (dvlb_header.magic_word != DVLBHeader::MAGIC_WORD) {
            std::stringstream stream;
            stream << "Wrong DVLB magic word: Got 0x" << std::hex << dvlb_header.magic_word;
            throw stream.str();
        }

        dvle_offsets.resize(dvlb_header.num_programs);
        dvle_headers.resize(dvlb_header.num_programs);
        for (auto& offset : dvle_offsets) {
            file.read((char*)&offset, sizeof(offset));
        }

        // DVLP comes directly after the DVLE offset table
        dvlp_offset = file.tellg();
        file.seekg(dvlp_offset);
        file.read((char*)&dvlp_header, sizeof(dvlp_header));
        if (dvlp_header.magic_word != DVLPHeader::MAGIC_WORD) {
            std::stringstream stream;
            stream << "Wrong DVLP magic word: Got " << std::hex << dvlp_header.magic_word;
            throw stream.str();
        }

        for (int i = 0; i < dvlb_header.num_programs; ++i) {
            auto& dvle_header = dvle_headers[i];
            file.seekg(dvle_offsets[i]);
            file.read((char*)&dvle_header, sizeof(dvle_header));
            if (dvle_header.magic_word != DVLEHeader::MAGIC_WORD) {
                std::stringstream stream;
                stream << "Wrong DVLE header in DVLE #" << i << ": " << std::hex << dvle_header.magic_word;
                throw stream.str();
            }
        }

        // TODO: Is there indeed exactly one filename per DVLE?
        dvle_filenames.resize(dvlb_header.num_programs);
        uint32_t offset = dvlp_offset + dvlp_header.filename_symbol_offset;
        for (int i = 0; i < dvlb_header.num_programs; ++i) {
            auto& filename = dvle_filenames[i];
            filename = ReadSymbol(offset);
            offset += filename.length() + 1;
        }

        // TODO: Proper size restriction?
        swizzle_info.resize(512);
        file.seekg(dvlp_offset + dvlp_header.swizzle_info_offset);
        file.read((char*)swizzle_info.data(), dvlp_header.swizzle_info_num_entries * sizeof(SwizzleInfo));
    }

    void ReadDVLE(int dvle_index) {
        // TODO: Check if we have called ReadHeaders() before!

        if (dvle_index >= dvlb_header.num_programs) {
            std::stringstream stream;
            stream << "Invalid DVLE index " << dvle_index << "given";
            throw stream.str();
        }

        auto& dvle_header = dvle_headers[dvle_index];
        auto& dvle_offset = dvle_offsets[dvle_index];

        uint32_t symbol_table_offset = dvle_offset + dvle_header.symbol_table_offset;

        constant_table.resize(dvle_header.constant_table_size);
        uint32_t constant_table_offset = dvle_offset + dvle_header.constant_table_offset;
        file.seekg(constant_table_offset);
        for (int i = 0; i < dvle_header.constant_table_size; ++i)
            file.read((char*)&constant_table[i], sizeof(ConstantInfo));

        label_table.resize(dvle_header.label_table_size);
        uint32_t label_table_offset = dvle_offset + dvle_header.label_table_offset;
        file.seekg(label_table_offset);
        for (int i = 0; i < dvle_header.label_table_size; ++i)
            file.read((char*)&label_table[i], sizeof(LabelInfo));
        for (const auto& label_info : label_table)
            labels.insert({label_info.program_offset, ReadSymbol(symbol_table_offset + label_info.name_offset)});

        output_register_info.resize(dvle_header.output_register_table_size);
        file.seekg(dvle_offset + dvle_header.output_register_table_offset);
        for (auto& info : output_register_info)
            file.read((char*)&info, sizeof(OutputRegisterInfo));

        uniform_table.resize(dvle_header.uniform_table_size);
        uint32_t uniform_table_offset = dvle_offset + dvle_header.uniform_table_offset;
        file.seekg(uniform_table_offset);
        for (int i = 0; i < dvle_header.uniform_table_size; ++i)
            file.read((char*)&uniform_table[i].basic, sizeof(uniform_table[i].basic));
        for (auto& uniform_info : uniform_table)
            uniform_info.name = ReadSymbol(symbol_table_offset + uniform_info.basic.symbol_offset);

        main_offset = dvlp_offset + dvlp_header.binary_offset;
    }

    const DVLBHeader& GetDVLBHeader() const {
        return dvlb_header;
    }

    const DVLPHeader& GetDVLPHeader() const {
        return dvlp_header;
    }

    const DVLEHeader& GetDVLEHeader(int index) const {
        return dvle_headers[index];
    }

    const std::string& GetFilename(int dvle_index) const {
        return dvle_filenames[dvle_index];
    }

    bool HasLabel(uint32_t offset) {
        return labels.find(offset) != labels.end();
    }

    std::string GetLabel (uint32_t offset) {
        auto it = labels.find(offset);
        if (it != labels.end())
            return it->second;
        return "";
    }

    Instruction ReadShaderInstruction(int word_index) {
        Instruction instr;
        file.seekg(main_offset + 4 * word_index);
        file.read((char*)&instr, sizeof(instr));
        return instr;
    }

    template<typename T>
    std::string LookupDestName(const T& dest, const SwizzlePattern& swizzle) {
        if (dest < 0x8) {
            // TODO: This one still needs some prettification in case
            //       multiple output_infos describing this output register
            //       are found.
            std::string ret;
            for (const auto& output_info : output_register_info) {
                if (dest != output_info.id)
                    continue;

                // Only display output register name if the output components it's mapped to are
                // actually written to.
                // swizzle.dest_mask and output_info.component_mask use different bit order,
                // so we can't use AND them bitwise to check this.
                int matching_mask = 0;
                for (int i = 0; i < 4; ++i)
                    matching_mask |= output_info.component_mask & (swizzle.DestComponentEnabled(i) << i);

                if (!matching_mask)
                    continue;

                // Add a vertical bar so that we have at least *some*
                // indication that we hit multiple matches.
                if (!ret.empty())
                    ret += "|";

                ret += output_info.GetFullName();
            }
            if (!ret.empty())
                return ret;
        } else if (dest.GetRegisterType() == Instruction::Temporary) {
            // TODO: Not sure if uniform_info can assign names to temporary registers.
            //       If that is the case, we should check the table for better names here.
            std::stringstream stream;
            stream << "temp_" << std::hex << dest.GetIndex();
            return stream.str();
        }
        return "(?)";
    }

    template<class T>
    std::string LookupSourceName(const T& source, unsigned addr_reg_index) {
        if (source.GetRegisterType() != Instruction::Temporary) {
            for (const auto& uniform_info : uniform_table) {
                // Magic numbers are needed because uniform info registers use the
                // range 0..0x10 for input registers and 0x10...0x70 for uniform registers,
                // i.e. there is a "gap" at the temporary registers, for which no
                // name can be assigned (?).
                int off = (source.GetRegisterType() == Instruction::Input) ? 0 : 0x10;
                if (source - off >= uniform_info.basic.reg_start &&
                    source - off <= uniform_info.basic.reg_end) {
                    std::string name = uniform_info.name;

                    std::string index;
                    bool is_array = uniform_info.basic.reg_end != uniform_info.basic.reg_start;
                    if (is_array) {
                        index += std::to_string(source - off - uniform_info.basic.reg_start);
                    }
                    if (addr_reg_index != 0) {
                        index += (is_array) ? " + " : "";
                        index += "a" + std::to_string(addr_reg_index - 1);
                    }

                    if (!index.empty())
                        name += "[" + index +  "]";

                    return name;
                }
            }
        }

        // Constants and uniforms really are the same internally
        for (const auto& constant_info : constant_table) {
            if (source - 0x20 == constant_info.regid) {
                return "const_" + std::to_string(constant_info.regid.Value());
            }
        }

        // For temporary registers, we at least print "temp_X" if no better name could be found.
        if (source.GetRegisterType() == Instruction::Temporary) {
            std::stringstream stream;
            stream << "temp_" << std::hex << source.GetIndex();
            return stream.str();
        }

        return "(?)";
    }

private:

    // Reads a null-terminated string from the given offset
    std::string ReadSymbol(uint32_t offset) {
        std::string name;
        file.seekg(offset);
        std::getline(file, name, '\0');
        return name;
    };

    std::fstream file;


    DVLBHeader dvlb_header;
    DVLPHeader dvlp_header;

    uint32_t dvlp_offset;

    // TODO: Put this into a struct!
public:
    std::vector<SwizzleInfo> swizzle_info;

    std::vector<uint32_t>    dvle_offsets;
    std::vector<DVLEHeader>  dvle_headers;
    std::vector<std::string> dvle_filenames;

    std::vector<ConstantInfo> constant_table;
    std::vector<LabelInfo> label_table;
    std::map<uint32_t, std::string> labels;
    std::vector<OutputRegisterInfo> output_register_info;
    std::vector<UniformInfo> uniform_table;

    uint32_t main_offset;
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
        for (int i = 0; i < parser.constant_table.size(); ++i) {
            auto& info = parser.constant_table[i];
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

        for (int i = 0; i < parser.label_table.size(); ++i) {
            const auto& label_info = parser.label_table[i];
            std::cout << "Found label \"" << parser.labels[label_info.program_offset]
                      << "\" at program offset 0x" << std::hex << 4 * label_info.program_offset
                      << std::endl;
        }

        for (auto& info : parser.output_register_info)
            std::cout << "Output register info:  o" << info.id.Value() << " = " << std::setw(13) << info.GetFullName() << " (" << std::hex << std::setw(16) << std::setfill('0') << (uint64_t)info.hex << std::setfill(' ') << ")" << std::endl;

        for (auto& uniform_info : parser.uniform_table)
//            std::cout << "Found uniform symbol \"" << std::setw(20) << uniform_info.name << "\" for registers 0x" << std::setfill('0') << std::setw(2) << uniform_info.basic.reg_start << "-0x" << std::setw(2) << uniform_info.basic.reg_end << " at offset 0x" << std::hex << symbol_table_offset + uniform_info.basic.symbol_offset << std::setfill(' ') << std::endl;
            std::cout << "Found uniform symbol \"" << std::setw(20) << uniform_info.name << "\" for registers 0x" << std::setfill('0') << std::setw(2) << uniform_info.basic.reg_start << "-0x" << std::setw(2) << uniform_info.basic.reg_end << std::setfill(' ') << std::endl;


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

    for (uint32_t word = 0; word < parser.GetDVLPHeader().binary_size_words; ++word) {
        std::cout.flags(std::ios::left | std::ios::hex);
        if (parser.HasLabel(word)) {
            std::cout << std::setw(8) << std::right << std::setfill('0') << 4*word
                      << " [--------] " << parser.GetLabel(word) << ":" << std::endl;
        }

        Instruction instr = parser.ReadShaderInstruction(word);

        std::cout << std::setw(8) << std::right << std::setfill('0') << 4*word << " "
                  << "[" << std::setw(8) << std::right << std::setfill('0') << instr.hex << "]     "
                  << std::setw(7) << std::left << std::setfill(' ') << instr.opcode.GetInfo().name;

        const SwizzlePattern& swizzle = parser.swizzle_info[instr.common.operand_desc_id].pattern;

        // TODO: Not sure if name lookup works properly, yet!

        if (instr.opcode.GetInfo().type == Instruction::OpCodeType::Arithmetic) {
            std::string src1_relative_address;
            if (!instr.common.AddressRegisterName().empty())
                src1_relative_address = "[" + instr.common.AddressRegisterName() + "]";

            std::cout << std::setw(4) << std::right << instr.common.dest.GetName() << "." << swizzle.DestMaskToString() << "  "
                      << std::setw(8) << std::right << ((swizzle.negate_src1 ? "-" : "") + instr.common.src1.GetName()) + src1_relative_address << "." << swizzle.SelectorToString(false) << "  ";

            if (instr.opcode.GetInfo().subtype & Instruction::OpCodeInfo::Src2)
                std::cout << std::setw(4) << std::right << (swizzle.negate_src2 ? "-" : "") + instr.common.src2.GetName() << "." << swizzle.SelectorToString(true) << "   ";
            else
                std::cout << "            ";

            std::cout << std::setw(2) << instr.common.operand_desc_id.Value() << " addr:" << instr.common.address_register_index.Value()
                      << ";      " << parser.LookupDestName(instr.common.dest, swizzle) << " <- " << (swizzle.negate_src1 ? "-" : "") + parser.LookupSourceName(instr.common.src1, instr.common.address_register_index);
            if (instr.opcode.GetInfo().subtype & Instruction::OpCodeInfo::Src2)
                std::cout << ", " << (swizzle.negate_src2 ? "-" : "") + parser.LookupSourceName(instr.common.src2, 0);

            std::cout << std::endl;
        } else if (instr.opcode.GetInfo().type == Instruction::OpCodeType::Conditional) {
            bool has_neg_x = instr.opcode.GetInfo().subtype & Instruction::OpCodeInfo::NegX;
            bool has_neg_y = instr.opcode.GetInfo().subtype & Instruction::OpCodeInfo::NegY;
            const char* ops[] = {
                " || ", " && ", "", ""
            };
            bool show_x = instr.conditional.op != instr.conditional.JustY;
            bool show_y = instr.conditional.op != instr.conditional.JustX;
            if (instr.opcode.GetInfo().subtype & Instruction::OpCodeInfo::JustCondition) {
                if (show_x)
                    std::cout << ((has_neg_x && instr.conditional.negx) ? "!" : " ") << "cc.x";
                std::cout << ops[instr.conditional.op];
                if (show_y)
                    std::cout << ((has_neg_y && instr.conditional.negy) ? "!" : " ") << "cc.y";

                std::cout << "  ";
            }

            if (instr.opcode.GetInfo().subtype & Instruction::OpCodeInfo::Dst) {
                std::cout << "to 0x" << std::setw(4) << std::right << std::setfill('0') << 4 * instr.conditional.dest_offset
                          << " aka \"" << parser.GetLabel(instr.conditional.dest_offset) << "\"";
            }

            if (instr.opcode.GetInfo().subtype & Instruction::OpCodeInfo::Num) {
                // TODO: This is actually "Up till exclusively"
                std::cout << " to 0x" << std::setw(4) << std::right << std::setfill('0') << 4 * instr.conditional.dest_offset + 4 * instr.conditional.num_instructions + 4
                          << " aka \"" << parser.GetLabel(instr.conditional.dest_offset + instr.conditional.num_instructions + 1) << "\"";
            }

            std::cout << std::endl;
        } else {
            std::cout << std::endl;
        }
    }

    std::cout << std::endl << "Swizzle patterns:" << std::endl;

    for (int i = 0; i < parser.GetDVLPHeader().swizzle_info_num_entries; ++i) {
        const auto& info = parser.swizzle_info[i];
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
