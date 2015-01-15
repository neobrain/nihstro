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

#include <fstream>
#include <iostream>
#include <iterator>

#include "nihstro/inline_assembly.h"

using namespace nihstro;

static const auto in_pos = SourceRegister::MakeInput(0);
static const auto in_tex = SourceRegister::MakeInput(1);
static const auto in_norm = SourceRegister::MakeInput(2);
static const auto backup_pos = SourceRegister::MakeTemporary(1);
static const auto temp_pos = SourceRegister::MakeTemporary(0);

static const auto constant = SourceRegister::MakeFloat(20);

static const SourceRegister projection[4] = { SourceRegister::MakeFloat(0), SourceRegister::MakeFloat(1), SourceRegister::MakeFloat(2), SourceRegister::MakeFloat(3) };
static const SourceRegister modelview[4] = { SourceRegister::MakeFloat(4), SourceRegister::MakeFloat(5), SourceRegister::MakeFloat(6), SourceRegister::MakeFloat(7) };
static const auto light_direction = SourceRegister::MakeFloat(8);
static const auto light_ambient = SourceRegister::MakeFloat(9);

static const DestRegister out_pos = DestRegister::MakeOutput(0);
static const DestRegister out_col = DestRegister::MakeOutput(1);
static const DestRegister out_tex0 = DestRegister::MakeOutput(2);
static const DestRegister out_tex1 = DestRegister::MakeOutput(3);
static const DestRegister out_tex2 = DestRegister::MakeOutput(4);

const auto shbin = InlineAsm::CompileToShbin({
    // TODO: Declare output names
    // TODO: Declare constant
    // TODO: Declare uniform names
    // TODO: Explicitly set entry point
    InlineAsm::DeclareOutput(out_pos, OutputRegisterInfo::POSITION),
    InlineAsm::DeclareOutput(out_col, OutputRegisterInfo::COLOR),
    InlineAsm::DeclareOutput(out_tex0, OutputRegisterInfo::TEXCOORD0),
    InlineAsm::DeclareOutput(out_tex1, OutputRegisterInfo::TEXCOORD1),
    InlineAsm::DeclareOutput(out_tex2, OutputRegisterInfo::TEXCOORD2),

    InlineAsm::DeclareConstant(constant, 1.0, 0.0, 0.5, 1.0),

    InlineAsm::DeclareUniform(projection[0], projection[3], "projection"),
    InlineAsm::DeclareUniform(modelview[0], modelview[3], "modelview"),
    InlineAsm::DeclareUniform(light_direction, light_direction, "lightDirection"),
    InlineAsm::DeclareUniform(light_ambient, light_ambient, "lightAmbient"),

    { OpCode::Id::MOV, backup_pos, "xyz", in_pos, "xyz" },
    { OpCode::Id::MOV, backup_pos, "w", constant, "xyzw" }, // TODO: Would like to just specify "w" here! // TODO: Somehow, c4 gets written instead...

    { OpCode::Id::DP4, temp_pos, "x", modelview[0], backup_pos },
    { OpCode::Id::DP4, temp_pos, "y", modelview[1], backup_pos },
    { OpCode::Id::DP4, temp_pos, "z", modelview[2], backup_pos },
    { OpCode::Id::MOV, temp_pos, "w", constant, "xyzw" }, // TODO: Would like to just specify "w" here!

    { OpCode::Id::DP4, out_pos, "x", projection[0], temp_pos },
    { OpCode::Id::DP4, out_pos, "y", projection[1], temp_pos },
    { OpCode::Id::DP4, out_pos, "z", projection[2], temp_pos },
    { OpCode::Id::DP4, out_pos, "w", projection[3], temp_pos },

    { OpCode::Id::MOV, out_tex0, in_tex },
    { OpCode::Id::MOV, out_tex1, constant, "yyyw" },
    { OpCode::Id::MOV, out_tex2, constant, "yyyw" },

    { OpCode::Id::DP3, temp_pos, "xyz", light_direction, in_norm },
    { OpCode::Id::MAX, temp_pos, "xyz", constant, "yyy", temp_pos },
    { OpCode::Id::MUL, temp_pos, "xyz", light_ambient, "www", temp_pos },
    { OpCode::Id::ADD, out_col, "xyz", light_ambient, temp_pos },
    { OpCode::Id::MOV, out_col, "w", constant, "xyzw" }, // TODO: Would like to just specify "w" here!

    { OpCode::Id::NOP },
    { OpCode::Id::END }
});

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cout << "Error: No filename given" << std::endl;
        return 0;
    }

    std::ofstream file(argv[1], std::ios::binary);
    std::copy(shbin.begin(), shbin.end(), std::ostream_iterator<uint8_t>(file));

    std::cout << "Successfully compiled shader to " << argv[1] << "!" << std::endl;

	return 0;
}
