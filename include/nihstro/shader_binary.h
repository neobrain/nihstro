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

#include "shader_bytecode.h"

namespace nihstro {

#pragma pack(1)
struct DVLBHeader {
    enum : uint32_t {
        MAGIC_WORD = 0x424C5644, // "DVLB"
    };

    uint32_t magic_word;
    uint32_t num_programs;

    // DVLE offset table with num_programs entries follows
};
static_assert(sizeof(DVLBHeader) == 0x8, "Incorrect structure size");

struct DVLPHeader {
    enum : uint32_t {
        MAGIC_WORD = 0x504C5644, // "DVLP"
    };

    uint32_t magic_word;
    uint32_t version;
    uint32_t binary_offset;  // relative to DVLP start
    uint32_t binary_size_words;
    uint32_t swizzle_info_offset;
    uint32_t swizzle_info_num_entries;
    uint32_t filename_symbol_offset;
};
static_assert(sizeof(DVLPHeader) == 0x1C, "Incorrect structure size");

struct DVLEHeader {
    enum : uint32_t {
        MAGIC_WORD = 0x454c5644, // "DVLE"
    };

    enum class ShaderType : uint8_t {
        VERTEX = 0,
        GEOMETRY = 1,
    };

    uint32_t magic_word;
    uint16_t pad1;
    ShaderType type;
    uint8_t pad2;

    // Offset within binary blob to program entry point
    uint32_t main_offset_words;
    uint32_t endmain_offset_words;

    uint32_t pad3;
    uint32_t pad4;

    // Table of constant values for single registers
    uint32_t constant_table_offset;
    uint32_t constant_table_size; // number of entries

    // Table of program code labels
    uint32_t label_table_offset;
    uint32_t label_table_size;

    // Table of output registers and their semantics
    uint32_t output_register_table_offset;
    uint32_t output_register_table_size;

    // Table of uniforms (which may span multiple registers) and their values
    uint32_t uniform_table_offset;
    uint32_t uniform_table_size;

    // Table of null-terminated strings referenced by the tables above
    uint32_t symbol_table_offset;
    uint32_t symbol_table_size;

};
static_assert(sizeof(DVLEHeader) == 0x40, "Incorrect structure size");


struct SwizzleInfo {
    SwizzlePattern pattern;
    uint32_t unknown;
};

struct ConstantInfo {
    union {
        // This field is a custom extension, and hence NOT OFFICIALLY SUPPORTED
        // It was only added to support loading citra's shader dumps until citra
        // can convert floats to float24!
        BitField<0, 1, uint32_t> is_float32;

        BitField<16, 8, uint32_t> regid;
        uint32_t full_first_word;
    };

    // float24 values..
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t w;
};

struct LabelInfo {
    BitField<0, 8, uint32_t> id;
    uint32_t program_offset;
    uint32_t unk;
    uint32_t name_offset;
};

union OutputRegisterInfo {
    enum Type : uint64_t {
        POSITION = 0,
        COLOR = 2,
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

#pragma pack()

} // namespace
