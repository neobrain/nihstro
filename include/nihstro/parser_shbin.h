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

#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <vector>

#include "nihstro/shader_binary.h"

namespace nihstro {

struct ShaderInfo {
    std::vector<Instruction> code;
    std::vector<SwizzleInfo> swizzle_info;

    std::vector<ConstantInfo> constant_table;
    std::vector<LabelInfo> label_table;
    std::map<uint32_t, std::string> labels;
    std::vector<OutputRegisterInfo> output_register_info;
    std::vector<UniformInfo> uniform_table;

    void Clear() {
        code.clear();
        swizzle_info.clear();
        constant_table.clear();
        label_table.clear();
        labels.clear();
        output_register_info.clear();
        uniform_table.clear();
    }

    bool HasLabel(uint32_t offset) const {
        return labels.find(offset) != labels.end();
    }

    std::string GetLabel (uint32_t offset) const {
        auto it = labels.find(offset);
        if (it != labels.end())
            return it->second;
        return "";
    }

    template<typename T>
    std::string LookupDestName(const T& dest, const SwizzlePattern& swizzle) const {
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

                ret += output_info.GetSemanticName();
            }
            if (!ret.empty())
                return ret;
        } else if (dest.GetRegisterType() == RegisterType::Temporary) {
            // TODO: Not sure if uniform_info can assign names to temporary registers.
            //       If that is the case, we should check the table for better names here.
            std::stringstream stream;
            stream << "temp_" << std::hex << dest.GetIndex();
            return stream.str();
        }
        return "(?)";
    }

    template<class T>
    std::string LookupSourceName(const T& source, unsigned addr_reg_index) const {
        if (source.GetRegisterType() != RegisterType::Temporary) {
            for (const auto& uniform_info : uniform_table) {
                // Magic numbers are needed because uniform info registers use the
                // range 0..0x10 for input registers and 0x10...0x70 for uniform registers,
                // i.e. there is a "gap" at the temporary registers, for which no
                // name can be assigned (?).
                int off = (source.GetRegisterType() == RegisterType::Input) ? 0 : 0x10;
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
        if (source.GetRegisterType() == RegisterType::Temporary) {
            std::stringstream stream;
            stream << "temp_" << std::hex << source.GetIndex();
            return stream.str();
        }

        return "(?)";
    }
};

class ShbinParser {
public:
    void ReadHeaders(const std::string& filename);

    void ReadDVLE(int dvle_index);

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

private:

    // Reads a null-terminated string from the given offset
    std::string ReadSymbol(uint32_t offset);

    std::fstream file;


    DVLBHeader dvlb_header;
    DVLPHeader dvlp_header;

    uint32_t dvlp_offset;

public:
    std::vector<uint32_t>    dvle_offsets;
    std::vector<DVLEHeader>  dvle_headers;
    std::vector<std::string> dvle_filenames;

    ShaderInfo shader_info;

    uint32_t main_offset;
};


} // namespace
