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

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "nihstro/parser_assembly.h"

#include "nihstro/shader_binary.h"
#include "nihstro/shader_bytecode.h"

using namespace nihstro;

enum class RegisterSpace : int {
    Input        = 0,
    Temporary    = 0x10,
    FloatUniform = 0x20,
    Output       = 0x80,
    Address      = 0x90,
    AddressEnd   = 0x92,

    Max          = 0x92,
};

// Smallest unit an expression evaluates to:
// Index to register + number of components + swizzle mask + sign
// Labels are different.
struct Atomic {
    int register_index; // TODO: Change type to RegisterSpace
    InputSwizzlerMask mask;
    bool negate;
    int relative_address_source;

    const RegisterType GetType() const {
        if (register_index >= (int)RegisterSpace::Address)
            return RegisterType::Address;
        else if (register_index >= (int)RegisterSpace::Output)
            return RegisterType::Output;
        else if (register_index >= (int)RegisterSpace::FloatUniform)
            return RegisterType::FloatUniform;
        else if (register_index >= (int)RegisterSpace::Temporary)
            return RegisterType::Temporary;
        else if (register_index >= (int)RegisterSpace::Input)
            return RegisterType::Input;
    }

    int GetIndex() const {
        if (register_index >= (int)RegisterSpace::Address)
            return register_index - (int)RegisterSpace::Address;
        else if (register_index >= (int)RegisterSpace::Output)
            return register_index - (int)RegisterSpace::Output;
        else if (register_index >= (int)RegisterSpace::FloatUniform)
            return register_index - (int)RegisterSpace::FloatUniform;
        else if (register_index >= (int)RegisterSpace::Temporary)
            return register_index - (int)RegisterSpace::Temporary;
        else if (register_index >= (int)RegisterSpace::Input)
            return register_index - (int)RegisterSpace::Input;
    }
};

// TODO: Support labels as identifiers...
std::vector<Atomic> identifiers;

struct DestSwizzlerMask {
    DestSwizzlerMask(const InputSwizzlerMask& input) : component_set{false, false, false, false} {
        for (InputSwizzlerMask::Component comp : {InputSwizzlerMask::x, InputSwizzlerMask::y,
                                                  InputSwizzlerMask::z, InputSwizzlerMask::w}) {
            for (int i = 0; i < input.num_components; ++i) {
                if (comp == input.components[i]) {
                    component_set[comp] = true;
                }
            }
        }
    }

    bool component_set[4];
};

struct SourceSwizzlerMask {

    // Generate source mask according to the layout given by the destination mask
    // E.g. the source swizzle pattern used by the instruction "mov o0.zw, t0.xy" will
    // be {(undefined),(undefined),x,y} rather than {x,y,(undefined),(undefined)}.
    static SourceSwizzlerMask AccordingToDestMask(const InputSwizzlerMask& input, const DestSwizzlerMask& dest) {
        SourceSwizzlerMask ret = {Unspecified, Unspecified, Unspecified, Unspecified };

        int active_component = 0;
        for (int i = 0; i < 4; ++i)
            if (dest.component_set[i])
                ret.components[i] = static_cast<Component>(input.components[active_component++]);

        return ret;
    }

    static SourceSwizzlerMask Expand(const InputSwizzlerMask& input) {
        SourceSwizzlerMask ret = {Unspecified, Unspecified, Unspecified, Unspecified };

        for (int i = 0; i < input.num_components; ++i)
            ret.components[i] = static_cast<Component>(input.components[i]);

        return ret;
    }

    enum Component : uint8_t {
        x = 0,
        y = 1,
        z = 2,
        w = 3,
        Unspecified
    };
    Component components[4];
};

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "No output filename given" << std::endl;
        return 1;
    }

    if (argc < 3) {
        std::cerr << "No input filenames given" << std::endl;
        return 1;
    }

    std::string input_code;
    std::string::iterator begin;
    std::string::iterator preparse_begin;

    try {

    std::string output_filename = argv[1];
    std::string input_filename = argv[2];

    std::ifstream input_file(input_filename);
    if (input_file)
    {
        input_file.seekg(0, std::ios::end);
        input_code.resize(input_file.tellg());
        input_file.seekg(0, std::ios::beg);
        input_file.read(&input_code[0], input_code.size());
        input_file.close();
    }
    else
    {
        throw "Could not open input file";
    }

    begin = input_code.begin();

    std::vector<Instruction> instructions;
    std::vector<SwizzlePattern> swizzle_patterns;

    struct CustomLabelInfo {
        uint32_t program_offset;
        uint32_t symbol_table_index;
    };

    std::vector<CustomLabelInfo> label_table;

    std::vector<std::string> symbol_table;

    uint32_t program_write_offset = 0;

    ParserContext context;
    for (int i = 0; i <= (int)RegisterSpace::Max; ++i) {
        identifiers.push_back({i, InputSwizzlerMask::FullMask()});

        std::string name;
        if (i == (int)RegisterSpace::AddressEnd)
            name = "lcnt";
        else if (i >= (int)RegisterSpace::Address)
            name = "a" + std::to_string(i - (int)RegisterSpace::Address);
        else if (i >= (int)RegisterSpace::Output)
            name = "o" + std::to_string(i - (int)RegisterSpace::Output);
        else if (i >= (int)RegisterSpace::FloatUniform)
            name = "c" + std::to_string(i - (int)RegisterSpace::FloatUniform);
        else if (i >= (int)RegisterSpace::Temporary)
            name = "r" + std::to_string(i - (int)RegisterSpace::Temporary);
        else if (i >= (int)RegisterSpace::Input)
            name = "v" + std::to_string(i - (int)RegisterSpace::Input);

        assert(!name.empty());

        context.identifiers.add(name, i);
    }


    while (true) {
        Parser parser(context);
        StatementLabel statement_label;
        StatementInstruction statement_instruction;
        StatementDeclaration statement_declaration;

        // First off, move iterator past preceding comments, blanks, etc
        parser.Skip(begin, input_code.end());

        // Now perform the actual parsing
        preparse_begin = begin;
        if (parser.ParseLabel(begin, input_code.end(), &statement_label)) {
            std::string label_symbol = statement_label;

            auto it = std::find(symbol_table.begin(), symbol_table.end(), label_symbol);
            if (it != symbol_table.end())
                throw "Label \"" + label_symbol + "\" already defined in symbol table";

            symbol_table.push_back(label_symbol);
            uint32_t symbol_table_index = symbol_table.size() - 1;

            CustomLabelInfo label_info = { program_write_offset, symbol_table_index };
            label_table.push_back(label_info);
        } else if (parser.ParseInstruction(begin, input_code.end(), &statement_instruction)) {
            auto& instr = statement_instruction;

            Instruction shinst;
            shinst.hex = 0;
            shinst.opcode.Assign(instr.GetOpCode());
            const std::vector<Expression>& args = instr.GetArguments();
            std::vector<Atomic> arguments;
            for (const auto& expr : args) {
                auto EvaluateExpression = [](const Expression& expr) {
                    Atomic ret = identifiers[expr.GetIdentifier()];

                    ret.negate = expr.GetSign() == -1;
                    ret.relative_address_source = 0;

                    bool relative_address_set = false;
                    if (expr.HasIndexExpression()) {
                        const auto& array_index_expression = expr.GetIndexExpression();
                        int index = 0;
                        for (int i = 0; i < array_index_expression.GetCount(); ++i) {
                            if (array_index_expression.IsRawIndex(i)) {
                                index += array_index_expression.GetRawIndex(i);
                            } else if (array_index_expression.IsAddressRegisterIdentifier(i)) {
                                if (relative_address_set) {
                                    throw "May not use more than one register in relative addressing";
                                }

                                // TODO: Make sure the referenced identifier is not completely bogus
                                ret.relative_address_source = identifiers[array_index_expression.GetAddressRegisterIdentifier(i)].register_index;
                                if (ret.relative_address_source < (int)RegisterSpace::Address ||
                                    ret.relative_address_source > (int)RegisterSpace::AddressEnd) {
                                    throw "Invalid register " + std::to_string(array_index_expression.GetAddressRegisterIdentifier(i))+ " " + std::to_string(ret.relative_address_source) + " used for relative addressing (only a0, a1 and lcnt are valid indexes)";
                                }
                                ret.relative_address_source -= (int)RegisterSpace::Address;
                                relative_address_set = true;
                            }
                        }
                        ret.register_index += index;
                    }

                    // Apply swizzle mask(s)
                    for (const auto& swizzle_mask : expr.GetSwizzleMasks()) {
                        // TODO: Error out if the swizzle masks can't actually be merged..

                        InputSwizzlerMask out;
                        out.num_components = swizzle_mask.num_components;
                        for (int comp = 0; comp < swizzle_mask.num_components; ++comp) {
                            out.components[comp] = ret.mask.components[swizzle_mask.components[comp]];
                        }
                        ret.mask = out;
                    }
                    return ret;
                };
                arguments.push_back(EvaluateExpression(expr));
            }

            int num_args = args.size();
            switch (shinst.opcode.GetInfo().type) {
                case Instruction::OpCodeType::Arithmetic:
                {
                    const int num_inputs = shinst.opcode.GetInfo().NumArguments() - 1;
                    if (num_args != num_inputs + 1)
                        throw "Incorrect number of arguments. Expected " + std::to_string(num_inputs + 1) + ", got " + std::to_string(num_args);

                    auto AssertRegisterReadable = [](RegisterType type) {
                        if (type != RegisterType::Input && type != RegisterType::Temporary &&
                            type != RegisterType::FloatUniform)
                            throw "Specified register is not readable (only input, temporary and uniform registers are writeable)";
                    };
                    auto AssertRegisterWriteable = [](RegisterType type, int index) {
                        if (type != RegisterType::Output && type != RegisterType::Temporary)
                            throw "Specified register " + std::to_string((int)type) + " " + std::to_string(index) + " is not writeable (only output and temporary registers are writeable)";
                    };
                    AssertRegisterWriteable(arguments[0].GetType(), arguments[0].GetIndex());
                    AssertRegisterReadable(arguments[1].GetType());

                    // If no swizzler have been specified, use .xyzw - compile errors triggered by this are intended! (accessing subvectors should be done explicitly)
                    InputSwizzlerMask input_dest_mask = arguments[0].mask;
                    InputSwizzlerMask input_mask_src1;
                    InputSwizzlerMask input_mask_src2;
                    if (num_inputs > 1) {
                        AssertRegisterReadable(arguments[2].GetType());

                        if (arguments[1].GetType() == RegisterType::FloatUniform &&
                            arguments[2].GetType() == RegisterType::FloatUniform) {
                            throw "Not more than one input register may be a floating point uniform";
                        }

                        // If second argument is a floating point register, swap it to first place
                        if (arguments[2].GetType() == RegisterType::FloatUniform) {
                            boost::swap(arguments[1], arguments[2]);
                        }

                        shinst.common.src2.Value().InitializeFromTypeAndIndex(arguments[2].GetType(), arguments[2].GetIndex());
                        input_mask_src2 = arguments[2].mask;
                    }
                    input_mask_src1 = arguments[1].mask;

                    shinst.common.dest.InitializeFromTypeAndIndex(arguments[0].GetType(), arguments[0].GetIndex());
                    shinst.common.src1.Value().InitializeFromTypeAndIndex(arguments[1].GetType(), arguments[1].GetIndex());

                    const bool is_dot_product = (shinst.opcode == Instruction::OpCode::DP3 ||
                                                 shinst.opcode == Instruction::OpCode::DP4);

                    if (is_dot_product) {
                        int expected_input_length = (shinst.opcode == Instruction::OpCode::DP3) ? 3 : 4;
                        if (input_mask_src1.num_components != expected_input_length ||
                            input_mask_src2.num_components != expected_input_length)
                            throw "Input registers for dot product instructions need to use proper number of components";

                        // NOTE: dest can use any number of components for dot products

                    } else {
                        // Generic syntax checking.. we likely want to have more special cases in the future!
                        if (input_dest_mask.num_components != input_mask_src1.num_components ||
                            (num_inputs > 1 && input_mask_src1.num_components != input_mask_src2.num_components)) {
                            throw "Input registers need to use the same number of components as the output register!"
                                    + std::string("(dest: ") + std::to_string(input_dest_mask.num_components) + " components, "
                                    + std::string("src1: ") + std::to_string(input_mask_src1.num_components) + " components"
                                    + ((num_inputs > 1) ? std::string(", src2: ") + std::to_string(input_mask_src2.num_components) + " components)"
                                                        : std::string(")"));
                        }
                    }

                    // Build swizzle patterns
                    // TODO: In the case of "few arguments", we can re-use patterns created with
                    //       larger argument lists to optimize pattern count.
                    SwizzlePattern swizzle_pattern;
                    swizzle_pattern.hex = 0;

                    DestSwizzlerMask dest_mask{input_dest_mask};
                    SourceSwizzlerMask mask_src1;
                    SourceSwizzlerMask mask_src2;
                    if (is_dot_product) {
                        mask_src1 = SourceSwizzlerMask::Expand(input_mask_src1);
                        mask_src2 = SourceSwizzlerMask::Expand(input_mask_src2);
                    } else {
                        mask_src1 = SourceSwizzlerMask::AccordingToDestMask(input_mask_src1, dest_mask);
                        mask_src2 = SourceSwizzlerMask::AccordingToDestMask(input_mask_src2, dest_mask);
                    }

                    for (int i = 0, active_component = 0; i < 4; ++i) {
                        if (dest_mask.component_set[i])
                            swizzle_pattern.SetDestComponentEnabled(i, true);

                        if (mask_src1.components[i] != SourceSwizzlerMask::Unspecified)
                            swizzle_pattern.SetSelectorSrc1(i, static_cast<SwizzlePattern::Selector>(mask_src1.components[i]));

                        if (num_inputs > 1 && mask_src2.components[i] != SourceSwizzlerMask::Unspecified)
                            swizzle_pattern.SetSelectorSrc2(i, static_cast<SwizzlePattern::Selector>(mask_src2.components[i]));
                    }

                    swizzle_pattern.negate_src1 = arguments[1].negate;
                    if (num_inputs > 1)
                        swizzle_pattern.negate_src2 = arguments[2].negate;

                    auto it = std::find_if(swizzle_patterns.begin(), swizzle_patterns.end(),
                                            [&swizzle_pattern](const SwizzlePattern& val) { return val.hex == swizzle_pattern.hex; });
                    if (it == swizzle_patterns.end()) {
                        swizzle_patterns.push_back(swizzle_pattern);
                        it = swizzle_patterns.end() - 1;

                        if (swizzle_patterns.size() > 127)
                            throw "Limit of 127 swizzle patterns has been exhausted";
                    }
                    shinst.common.operand_desc_id = it - swizzle_patterns.begin();

                    instructions.push_back(shinst);
                    break;
                }

                default:
                    throw "Unknown instruction encountered";
                    break;
            }
            ++program_write_offset;
        } else if (parser.ParseDeclaration(begin, input_code.end(), &statement_declaration)) {
            auto& var = statement_declaration;

            // TODO: check if valid identifiers are passed as arguments.
            //       It e.g. shouldn't be possible to declare an input register as output.

            Identifier id;
            std::string idname;

            if (var.which() == 0) {
                auto& var2 = boost::get<DeclarationConstant>(var);
                id = boost::fusion::at_c<1>(var2);
                idname = boost::fusion::at_c<0>(var2);

                // TODO: Add to constant table
            } else if (var.which() == 1) {
                auto& var2 = boost::get<DeclarationOutput>(var);
                id = boost::fusion::at_c<1>(var2);
                idname = boost::fusion::at_c<0>(var2);

                // TODO: Add to output table
                // TODO: Make sure the declared output actually gets set (otherwise the GPU freezes)
            } else if (var.which() == 2) {
                auto& var2 = boost::get<DeclarationAlias>(var);
                id = boost::fusion::at_c<1>(var2);
                idname = boost::fusion::at_c<0>(var2);

                // TODO: Add to uniform table
            } else {
                // TODO: Better error handling..
                throw "meh";
            }

            Atomic ret = identifiers[id];
            Identifier new_identifier = identifiers.size();
            identifiers.push_back(ret);
            context.identifiers.add(idname, new_identifier);
        } else {
            break;
        }
    }

    // Error out if we didn't parse the full file
    if (begin != input_code.end()) {
        std::cerr << "Aborting due to parse error..." << std::endl; // + input_code.substr(begin - input_code.begin());
        exit(1);
    }

    auto GetSymbolTableEntryByteOffset = [&symbol_table](int index) {
        int offset = 0;
        for (int i = 0; i < index; ++i) {
            offset += symbol_table[i].length() + 1;
        }
        return offset;
    };

    uint32_t main_offset = [&]() {
                               for (auto& label : label_table) {
                                   if ("main" == symbol_table[label.symbol_table_index]) {
                                       return GetSymbolTableEntryByteOffset(label.symbol_table_index);
                                   }
                               }
                               throw "No main label specified";
                           }();

    struct StuffToWrite {
        uint8_t* pointer;
        uint32_t size;
    };
    std::vector<StuffToWrite> writing_queue;
    uint32_t write_offset = 0;

    auto QueueForWriting = [&writing_queue,&write_offset](uint8_t* pointer, uint32_t size) {
        writing_queue.push_back({pointer, size});
        uint32_t old_write_offset = write_offset;
        write_offset += size;
        return old_write_offset;
    };

    struct {
        DVLBHeader header;
        uint32_t dvle_offset;
    } dvlb{ {DVLBHeader::MAGIC_WORD, 1 } }; // 1 DVLE

    DVLPHeader dvlp{ DVLPHeader::MAGIC_WORD };
    DVLEHeader dvle{ DVLEHeader::MAGIC_WORD };

    QueueForWriting((uint8_t*)&dvlb, sizeof(dvlb));
    uint32_t dvlp_offset = QueueForWriting((uint8_t*)&dvlp, sizeof(dvlp));
    dvlb.dvle_offset = QueueForWriting((uint8_t*)&dvle, sizeof(dvle));

    // TODO: Reduce the amount of binary code written to relevant portions
    dvlp.binary_offset = write_offset - dvlp_offset;
    dvlp.binary_size_words = instructions.size();
    QueueForWriting((uint8_t*)instructions.data(), instructions.size() * sizeof(uint32_t));

    dvlp.swizzle_info_offset = write_offset - dvlp_offset;
    dvlp.swizzle_info_num_entries = swizzle_patterns.size();
    uint32_t dummy = 0;
    for (int i = 0; i < swizzle_patterns.size(); ++i) {
        QueueForWriting((uint8_t*)&swizzle_patterns[i], sizeof(swizzle_patterns[i]));
        QueueForWriting((uint8_t*)&dummy, sizeof(dummy));
    }

    dvle.main_offset_words = main_offset;

    dvle.label_table_offset = write_offset - dvlb.dvle_offset;
    dvle.label_table_size = label_table.size();
    std::vector<LabelInfo> final_label_table;
    final_label_table.resize(label_table.size());
    for (auto& label : label_table) {
        LabelInfo info;
        info.id = 0; // Not sure what this should be
        info.program_offset = label.program_offset;
        info.unk = 0; // Not sure what this should be
        info.name_offset = GetSymbolTableEntryByteOffset(label.symbol_table_index);
        final_label_table.push_back(info);

        QueueForWriting((uint8_t*)&final_label_table.back(), sizeof(LabelInfo));
    }

    dvle.symbol_table_offset = write_offset - dvlb.dvle_offset;;
    dvle.symbol_table_size = GetSymbolTableEntryByteOffset(symbol_table.size() - 1) + (symbol_table.back().length() + 1);
    for (auto& symbol : symbol_table)
        QueueForWriting((uint8_t*)symbol.c_str(), symbol.length() + 1);

    // Write data to file
    static int dump_index = 0;
    std::ofstream file(output_filename, std::ios_base::out | std::ios_base::binary);

    for (auto& chunk : writing_queue) {
        file.write((char*)chunk.pointer, chunk.size);
    }

    } catch(const std::string& err) {
        std::cerr << "Error: " << err << std::endl;
        std::cerr << "At: " << input_code.substr(preparse_begin - input_code.begin(), begin - preparse_begin) << std::endl;
        return 1;
    } catch(const char* err) {
        std::cerr << "Error: " << err << std::endl;
        std::cerr << "At: " << input_code.substr(preparse_begin - input_code.begin(), begin - preparse_begin) << std::endl;
        return 1;
    }

    return 0;
}
