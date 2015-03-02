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

#include "nihstro/parser_shbin.h"

using namespace nihstro;

void ShbinParser::ReadHeaders(const std::string& filename) {
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
        stream << "Wrong DVLP magic word at offset " << std::hex << dvlp_offset << ": Got " << std::hex << dvlp_header.magic_word;
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

    // Read shader binary code
    shader_info.code.resize(dvlp_header.binary_size_words);
    file.seekg(dvlp_offset + dvlp_header.binary_offset);
    file.read((char*)shader_info.code.data(), dvlp_header.binary_size_words * sizeof(Instruction));

    // Read operand descriptor table
    shader_info.swizzle_info.resize(dvlp_header.swizzle_info_num_entries);
    file.seekg(dvlp_offset + dvlp_header.swizzle_info_offset);
    file.read((char*)shader_info.swizzle_info.data(), dvlp_header.swizzle_info_num_entries * sizeof(SwizzleInfo));
}

void ShbinParser::ReadDVLE(int dvle_index) {
    // TODO: Check if we have called ReadHeaders() before!

    if (dvle_index >= dvlb_header.num_programs) {
        std::stringstream stream;
        stream << "Invalid DVLE index " << dvle_index << "given";
        throw stream.str();
    }

    auto& dvle_header = dvle_headers[dvle_index];
    auto& dvle_offset = dvle_offsets[dvle_index];

    uint32_t symbol_table_offset = dvle_offset + dvle_header.symbol_table_offset;

    shader_info.constant_table.resize(dvle_header.constant_table_size);
    uint32_t constant_table_offset = dvle_offset + dvle_header.constant_table_offset;
    file.seekg(constant_table_offset);
    for (int i = 0; i < dvle_header.constant_table_size; ++i)
        file.read((char*)&shader_info.constant_table[i], sizeof(ConstantInfo));

    shader_info.label_table.resize(dvle_header.label_table_size);
    uint32_t label_table_offset = dvle_offset + dvle_header.label_table_offset;
    file.seekg(label_table_offset);
    for (int i = 0; i < dvle_header.label_table_size; ++i)
        file.read((char*)&shader_info.label_table[i], sizeof(LabelInfo));
    for (const auto& label_info : shader_info.label_table)
        shader_info.labels.insert({label_info.program_offset, ReadSymbol(symbol_table_offset + label_info.name_offset)});

    shader_info.output_register_info.resize(dvle_header.output_register_table_size);
    file.seekg(dvle_offset + dvle_header.output_register_table_offset);
    for (auto& info : shader_info.output_register_info)
        file.read((char*)&info, sizeof(OutputRegisterInfo));

    shader_info.uniform_table.resize(dvle_header.uniform_table_size);
    uint32_t uniform_table_offset = dvle_offset + dvle_header.uniform_table_offset;
    file.seekg(uniform_table_offset);
    for (int i = 0; i < dvle_header.uniform_table_size; ++i)
        file.read((char*)&shader_info.uniform_table[i].basic, sizeof(shader_info.uniform_table[i].basic));
    for (auto& uniform_info : shader_info.uniform_table)
        uniform_info.name = ReadSymbol(symbol_table_offset + uniform_info.basic.symbol_offset);

    main_offset = dvlp_offset + dvlp_header.binary_offset;
}

std::string ShbinParser::ReadSymbol(uint32_t offset) {
    std::string name;
    file.seekg(offset);
    std::getline(file, name, '\0');
    return name;
};
