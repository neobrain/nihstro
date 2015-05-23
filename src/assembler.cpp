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

#include <stack>

#include <boost/range/adaptor/sliced.hpp>
#include <boost/range/algorithm/count_if.hpp>

#include "nihstro/parser_assembly.h"

#include "nihstro/shader_binary.h"
#include "nihstro/shader_bytecode.h"

#include "nihstro/float24.h"

using namespace nihstro;

enum class RegisterSpace : int {
    Input           = 0,
    Temporary       = 0x10,
    FloatUniform    = 0x20,
    Output          = 0x80,
    Address         = 0x90,
    AddressEnd      = 0x92,
    ConditionalCode = 0x93,
    IntUniform      = 0x94,
    BoolUniform     = 0x98,

    Max             = BoolUniform + 15,
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
        if (register_index >= (int)RegisterSpace::BoolUniform)
            return RegisterType::BoolUniform;
        else if (register_index >= (int)RegisterSpace::IntUniform)
            return RegisterType::IntUniform;
        else if (register_index >= (int)RegisterSpace::ConditionalCode)
            return RegisterType::ConditionalCode;
        else if (register_index >= (int)RegisterSpace::Address)
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
        if (register_index >= (int)RegisterSpace::BoolUniform)
            return register_index - (int)RegisterSpace::BoolUniform;
        else if (register_index >= (int)RegisterSpace::IntUniform)
            return register_index - (int)RegisterSpace::IntUniform;
        else if (register_index >= (int)RegisterSpace::ConditionalCode)
            return register_index - (int)RegisterSpace::ConditionalCode;
        else if (register_index >= (int)RegisterSpace::Address)
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

    // Returns whether this is a float uniform register OR uses relative addressing
    bool IsExtended() const {
        return GetType() == RegisterType::FloatUniform ||
               relative_address_source != 0;
    }
};

struct DestSwizzlerMask {
    DestSwizzlerMask(const InputSwizzlerMask& input) {
        std::fill(component_set, &component_set[4], false);
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

static InputSwizzlerMask MergeSwizzleMasks(const InputSwizzlerMask& inner_mask, const InputSwizzlerMask& outer_mask) {
    // TODO: Error out if the swizzle masks can't actually be merged..

    InputSwizzlerMask out;
    out.num_components = outer_mask.num_components;
    for (int comp = 0; comp < outer_mask.num_components; ++comp) {
        out.components[comp] = inner_mask.components[outer_mask.components[comp]];
    }

    return out;
}

static std::map<std::string, Atomic> identifier_table;

static Atomic& LookupIdentifier(const std::string& name) {
    auto it = identifier_table.find(name);
    if (it == identifier_table.end())
        throw "Unknown identifier \"" + name + "\"";

    return it->second;
}

// Evaluate expression to a particular Atomic
static Atomic EvaluateExpression(const Expression& expr) {
    Atomic ret = LookupIdentifier(expr.GetIdentifier());

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

                ret.relative_address_source = LookupIdentifier(array_index_expression.GetAddressRegisterIdentifier(i)).register_index;
                if (ret.relative_address_source < (int)RegisterSpace::Address ||
                    ret.relative_address_source > (int)RegisterSpace::AddressEnd) {
                    throw "Invalid register " + array_index_expression.GetAddressRegisterIdentifier(i)+ " (" + std::to_string(ret.relative_address_source) + ") used for relative addressing (only a0, a1 and lcnt are valid indexes)";
                }
                ret.relative_address_source -= (int)RegisterSpace::Address - 1; // 0 is reserved for "no dynamic indexing", hence the first address register gets value 1
                relative_address_set = true;
            }
        }
        ret.register_index += index;
    }
    // Apply swizzle mask(s)
    for (const auto& swizzle_mask : expr.GetSwizzleMasks())
        ret.mask = MergeSwizzleMasks(ret.mask, swizzle_mask);

    return ret;
};

// TODO: Provide optimized versions for functions without src2
// TODO: Support src3 inputs
static size_t FindOrAddSwizzlePattern(std::vector<SwizzlePattern>& swizzle_patterns,
                               const DestSwizzlerMask& dest_mask,
                               const SourceSwizzlerMask& mask_src1,
                               const SourceSwizzlerMask& mask_src2,
                               bool negate_src1, bool negate_src2) {
    SwizzlePattern swizzle_pattern;
    swizzle_pattern.hex = 0;

    for (int i = 0, active_component = 0; i < 4; ++i) {
        if (dest_mask.component_set[i])
            swizzle_pattern.SetDestComponentEnabled(i, true);

        if (mask_src1.components[i] != SourceSwizzlerMask::Unspecified)
            swizzle_pattern.SetSelectorSrc1(i, static_cast<SwizzlePattern::Selector>(mask_src1.components[i]));

        if (mask_src2.components[i] != SourceSwizzlerMask::Unspecified)
            swizzle_pattern.SetSelectorSrc2(i, static_cast<SwizzlePattern::Selector>(mask_src2.components[i]));
    }

    swizzle_pattern.negate_src1 = negate_src1;
    swizzle_pattern.negate_src2 = negate_src2;

    auto it = std::find_if(swizzle_patterns.begin(), swizzle_patterns.end(),
                            [&swizzle_pattern](const SwizzlePattern& val) { return val.hex == swizzle_pattern.hex; });
    if (it == swizzle_patterns.end()) {
        swizzle_patterns.push_back(swizzle_pattern);
        it = swizzle_patterns.end() - 1;

        if (swizzle_patterns.size() > 127)
            throw "Limit of 127 swizzle patterns has been exhausted";
    }

    return it - swizzle_patterns.begin();
};

static size_t FindOrAddSwizzlePattern(std::vector<SwizzlePattern>& swizzle_patterns,
                               const SourceSwizzlerMask& mask_src1,
                               const SourceSwizzlerMask& mask_src2,
                               bool negate_src1, bool negate_src2) {
    SwizzlePattern swizzle_pattern;
    swizzle_pattern.hex = 0;

    for (int i = 0, active_component = 0; i < 4; ++i) {
        if (mask_src1.components[i] != SourceSwizzlerMask::Unspecified)
            swizzle_pattern.SetSelectorSrc1(i, static_cast<SwizzlePattern::Selector>(mask_src1.components[i]));

        if (mask_src2.components[i] != SourceSwizzlerMask::Unspecified)
            swizzle_pattern.SetSelectorSrc2(i, static_cast<SwizzlePattern::Selector>(mask_src2.components[i]));
    }

    swizzle_pattern.negate_src1 = negate_src1;
    swizzle_pattern.negate_src2 = negate_src2;

    auto it = std::find_if(swizzle_patterns.begin(), swizzle_patterns.end(),
                            [&swizzle_pattern](const SwizzlePattern& val) { return val.hex == swizzle_pattern.hex; });
    if (it == swizzle_patterns.end()) {
        swizzle_patterns.push_back(swizzle_pattern);
        it = swizzle_patterns.end() - 1;

        if (swizzle_patterns.size() > 127)
            throw "Limit of 127 swizzle patterns has been exhausted";
    }

    return it - swizzle_patterns.begin();
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
    unsigned code_line = 0;

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
        std::cerr << "Could not open input file " << input_filename << std::endl;
        return 1;
    }

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

    try {

    std::vector<Instruction> instructions;
    std::vector<SwizzlePattern> swizzle_patterns;

    struct CustomLabelInfo {
        uint32_t program_offset;
        uint32_t symbol_table_index;
    };

    std::vector<CustomLabelInfo> label_table;

    std::vector<std::string> symbol_table;
    std::vector<OutputRegisterInfo> output_table;
    std::vector<ConstantInfo> constant_table;
    std::vector<UniformInfo> uniform_table;

    uint32_t program_write_offset = 0;

    ParserContext context;
    for (int i = 0; i <= (int)RegisterSpace::Max; ++i) {
        std::string name;
        if (i >= (int)RegisterSpace::BoolUniform)
            name = "b" + std::to_string(i - (int)RegisterSpace::BoolUniform);
        else if (i >= (int)RegisterSpace::IntUniform)
            name = "i" + std::to_string(i - (int)RegisterSpace::IntUniform);
        else if (i == (int)RegisterSpace::ConditionalCode)
            name = "cc";
        else if (i == (int)RegisterSpace::AddressEnd)
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

        identifier_table.insert({name, {i, InputSwizzlerMask::FullMask()}});
    }

    struct CallStackElement {
        OpCode original_opcode;
        unsigned instruction_index;
        unsigned alternative_position;
        unsigned end_position;
    };
    std::stack<CallStackElement> call_stack;

    auto LookupLableAddress = [&symbol_table, &label_table](const std::string& name) {
        for (unsigned label_index = 0; label_index < label_table.size(); ++label_index) {
            const auto& label_entry = label_table[label_index];

            if (symbol_table[label_entry.symbol_table_index] == name)
                return label_entry.program_offset;
        }

        throw "Unknown label";
    };

    // First off, build label table via preprocessing
    try {
    begin = input_code.begin();
    while (begin != input_code.end()) {
        Parser parser(context);
        StatementLabel statement_label;
        OpCode opcode;

        parser.Skip(begin, input_code.end());

        if (parser.ParseLabel(begin, input_code.end(), &statement_label)) {
            std::string label_symbol = statement_label;

            auto it = std::find(symbol_table.begin(), symbol_table.end(), label_symbol);
            if (it != symbol_table.end())
                throw "Label \"" + label_symbol + "\" already defined in symbol table";

            symbol_table.push_back(label_symbol);
            uint32_t symbol_table_index = symbol_table.size() - 1;

            CustomLabelInfo label_info = { program_write_offset, symbol_table_index };
            label_table.push_back(label_info);
        } else if (parser.ParseOpCode(begin, input_code.end(), &opcode)) {
            // Increment program address for anything which will generate a non-pseudo-instruction.
            if (static_cast<uint32_t>((OpCode::Id)opcode) < static_cast<uint32_t>(OpCode::Id::PSEUDO_INSTRUCTION_START) ||
                opcode == OpCode::Id::GEN_IF || opcode == OpCode::Id::GEN_CALL ||
                opcode == OpCode::Id::GEN_JMP) {
                ++program_write_offset;
            }

            // TODO: This might cause issues with trivial opcodes...
            parser.SkipSingleLine(begin, input_code.end());
        } else {
            // If it's neither a (recognized) instruction nor a label, skip this line and complain later if need be
            parser.SkipSingleLine(begin, input_code.end());
        }
    }
    } catch (...) {
        // TODO: "endmain" declarations followed by EOF don't get parsed currently. Not sure why, yet.
        std::cerr << "Error while parsing for labels. Did you format a label declaration incorrectly? Note that all label declarations need to be followed by a line break." << std::endl;
        exit(1);
    }


    begin = input_code.begin();
    preparse_begin = begin;

    Parser parser(context);
    program_write_offset = 0;
    while (begin != input_code.end()) {
        StatementLabel statement_label;
        FloatOpInstruction statement_instruction;
        CompareInstruction compare_instruction;
        FlowControlInstruction statement_flow_control;
        StatementDeclaration statement_declaration;
        OpCode statement_simple;

        // First off, move iterator past preceding comments, blanks, etc
        code_line += parser.Skip(begin, input_code.end());

        auto AssertRegisterReadable = [](RegisterType type) {
            if (type != RegisterType::Input && type != RegisterType::Temporary &&
                type != RegisterType::FloatUniform)
                throw "Specified register is not readable (only input, temporary and uniform registers are writeable)";
        };
        auto AssertRegisterWriteable = [](RegisterType type, int index) {
            if (type != RegisterType::Output && type != RegisterType::Temporary)
                throw "Specified register " + std::to_string((int)type) + " " + std::to_string(index) + " is not writeable (only output and temporary registers are writeable)";
        };

        ++code_line;

        // Now perform the actual parsing
        preparse_begin = begin;
        if (parser.ParseLabel(begin, input_code.end(), &statement_label)) {
            // Already handled above
        } else if (parser.ParseSimpleInstruction(begin, input_code.end(), &statement_simple)) {
            OpCode opcode = statement_simple;

            switch (opcode) {
            case OpCode::Id::NOP:
            case OpCode::Id::END:
            case OpCode::Id::EMIT:
            {
                Instruction shinst;
                shinst.hex = 0;
                shinst.opcode = opcode;
                instructions.push_back(shinst);
                ++program_write_offset;
                break;
            }

            case OpCode::Id::ELSE:
            {
                if (call_stack.empty())
                    throw "ELSE may not be used without prior IF!";

                auto& reference = call_stack.top();
                if (reference.original_opcode != OpCode::Id::GEN_IF)
                    throw "ELSE may not be used if current scope is not an IF-body";

                if (reference.alternative_position != -1)
                    throw "ELSE was already called for this IF statement!";

                reference.alternative_position = program_write_offset;
                break;
            }

            case OpCode::Id::ENDIF:
            {
                if (call_stack.empty())
                    throw "ENDIF may not be used without prior IF!";

                auto& reference = call_stack.top();
                reference.end_position = program_write_offset;

                if (reference.original_opcode != OpCode::Id::GEN_IF)
                    throw "ENDIF may not be used if the current scope is not an IF-body";

                // If no ELSE branch was set, set it to the ENDIF position
                if (reference.alternative_position == -1)
                    reference.alternative_position = reference.end_position;

                Instruction& shinst = instructions.at(reference.instruction_index);
                shinst.flow_control.dest_offset = reference.alternative_position;
                shinst.flow_control.num_instructions = reference.end_position - reference.alternative_position;

                call_stack.pop();

                break;
            }

            case OpCode::Id::ENDLOOP:
            {
                if (call_stack.empty())
                    throw "ENDLOOP may not be used without prior LOOP!";

                auto& reference = call_stack.top();
                reference.end_position = program_write_offset;

                if (reference.original_opcode != OpCode::Id::LOOP)
                    throw "ENDLOOP may not be used if the current scope is not an LOOP-body";

                // TODO: Assert that the LOOP body spans at least one instruction!

                Instruction& shinst = instructions.at(reference.instruction_index);
                shinst.flow_control.dest_offset = reference.end_position - 1; // Point to the last instruction of the body

                call_stack.pop();

                break;
            }

            default:
            {
                std::stringstream ss("Unknown opcode ");
                ss << static_cast<uint32_t>((OpCode::Id)opcode);
                throw ss.str();
            }
            }
        } else if (parser.ParseFloatOp(begin, input_code.end(), &statement_instruction)) {
            auto& instr = statement_instruction;

            Instruction shinst;
            shinst.hex = 0;
            shinst.opcode.Assign(instr.GetOpCode());
            const std::vector<Expression>& args = instr.GetArguments();
            std::vector<Atomic> arguments;
            for (const auto& expr : args) {
                arguments.push_back(EvaluateExpression(expr));
            }

            int num_args = args.size();
            OpCode opcode = shinst.opcode.Value();
            switch (opcode.GetInfo().type) {
                case OpCode::Type::Arithmetic:
                {
                    if (opcode == OpCode::Id::MAD || opcode == OpCode::Id::MOVA)
                        throw "MAD and MOVA are not supported, yet";

                    const int num_inputs = opcode.GetInfo().NumArguments() - 1;

                    AssertRegisterWriteable(arguments[0].GetType(), arguments[0].GetIndex());
                    AssertRegisterReadable(arguments[1].GetType());

                    // If no swizzler have been specified, use .xyzw - compile errors triggered by this are intended! (accessing subvectors should be done explicitly)
                    InputSwizzlerMask input_dest_mask = arguments[0].mask;
                    InputSwizzlerMask input_mask_src1;
                    InputSwizzlerMask input_mask_src2;

                    bool inverse_instruction_format = false;

                    // Make sure not more than one float uniform is used, and move it to src1 if need be
                    if (num_inputs > 1) {
                        AssertRegisterReadable(arguments[2].GetType());

                        if (boost::count_if(arguments | boost::adaptors::sliced(1,3), [](const Atomic& a) { return a.IsExtended(); }) == 2) {
                            throw "Not more than one input register may be a floating point uniform and/or use dynamic indexing";
                        }

                        // If second argument is a floating point register, swap it to first place
                        if (arguments[2].IsExtended()) {
                            switch (opcode) {
                            case OpCode::Id::ADD:
                            case OpCode::Id::DP3:
                            case OpCode::Id::DP4:
                            case OpCode::Id::MUL:
                            case OpCode::Id::MAX:
                            case OpCode::Id::MIN:
                                // Commutative operation, so just exchange arguments
                                boost::swap(arguments[1], arguments[2]);
                                break;

                            case OpCode::Id::MAD:
                                // Commutative in first two arguments, so just exchange arguments
                                boost::swap(arguments[1], arguments[2]);
                                break;

                            case OpCode::Id::DPH:
                                opcode = OpCode::Id::DPHI;
                                inverse_instruction_format = true;
                                break;

                            case OpCode::Id::SGE:
                                opcode = OpCode::Id::SGEI;
                                inverse_instruction_format = true;
                                break;

                            case OpCode::Id::SLT:
                                opcode = OpCode::Id::SLTI;
                                inverse_instruction_format = true;
                                break;

                            default:
                                throw "This opcode is not supported with a float uniform in second place. Change your code to put the float uniform in the first place, instead.";
                            }
                        }

                        if (inverse_instruction_format) {
                            shinst.common.src2i = SourceRegister::FromTypeAndIndex(arguments[2].GetType(), arguments[2].GetIndex());
                        } else {
                            shinst.common.src2 = SourceRegister::FromTypeAndIndex(arguments[2].GetType(), arguments[2].GetIndex());
                        }
                        input_mask_src2 = arguments[2].mask;
                    }

                    input_mask_src1 = arguments[1].mask;

                    shinst.common.dest = DestRegister::FromTypeAndIndex(arguments[0].GetType(), arguments[0].GetIndex());
                    if (inverse_instruction_format) {
                        shinst.common.src1i = SourceRegister::FromTypeAndIndex(arguments[1].GetType(), arguments[1].GetIndex());
                    } else {
                        shinst.common.src1  = SourceRegister::FromTypeAndIndex(arguments[1].GetType(), arguments[1].GetIndex());
                    }

                    shinst.common.address_register_index = (inverse_instruction_format)
                                                            ? arguments[2].relative_address_source
                                                            : arguments[1].relative_address_source;

                    const bool is_dot_product = (opcode == OpCode::Id::DP3 ||
                                                 opcode == OpCode::Id::DP4 ||
                                                 opcode == OpCode::Id::DPH);

                    if (is_dot_product) {
                        int expected_input_length1 = (opcode == OpCode::Id::DP3) ? 3 : 4;
                        int expected_input_length2 = (opcode == OpCode::Id::DP4) ? 4 : 3;
                        if (input_mask_src1.num_components != expected_input_length1 ||
                            input_mask_src2.num_components != expected_input_length2)
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
                    shinst.common.operand_desc_id = FindOrAddSwizzlePattern(swizzle_patterns, dest_mask, mask_src1, mask_src2, arguments[0].negate, arguments[1].negate);

                    instructions.push_back(shinst);
                    break;
                }
                default:
                    throw "Unknown instruction encountered";
                    break;
            }
            ++program_write_offset;
        } else if (parser.ParseCompare(begin, input_code.end(), &compare_instruction)) {
            Instruction shinst;
            shinst.hex = 0;
            shinst.opcode.Assign(compare_instruction.GetOpCode());

            Atomic src1 = EvaluateExpression(compare_instruction.GetSrc1());
            Atomic src2 = EvaluateExpression(compare_instruction.GetSrc2());

            AssertRegisterReadable(src1.GetType());
            AssertRegisterReadable(src2.GetType());

            // If no swizzler have been specified, use .xyzw - compile errors triggered by this are intended! (accessing subvectors should be done explicitly)
            InputSwizzlerMask input_mask_src1;
            InputSwizzlerMask input_mask_src2;

            // Make sure not more than one float uniform is used, and move it to src1 if need be
            if (boost::count_if(std::initializer_list<Atomic>{src1, src2}, [](const Atomic& a) { return a.IsExtended(); }) == 2) {
                throw "Not more than one input register may be a floating point uniform and/or use dynamic indexing.";
            }

            // If second argument is a floating point register, swap it to first place and invert the compare mode
            if (src2.IsExtended()) {
                using OpType = Instruction::Common::CompareOpType;
                static auto InvertCompareMode = [](OpType::Op op) {
                    if (op == OpType::LessThan)
                        return OpType::GreaterThan;
                    else if (op == OpType::LessEqual)
                        return OpType::GreaterEqual;
                    else if (op == OpType::GreaterThan)
                        return OpType::LessThan;
                    else if (op == OpType::GreaterEqual)
                        return OpType::LessEqual;
                    else
                        return op;
                };
                shinst.common.compare_op.x = InvertCompareMode(compare_instruction.GetOp1());
                shinst.common.compare_op.y = InvertCompareMode(compare_instruction.GetOp2());

                boost::swap(src1, src2);
            } else {
                shinst.common.compare_op.x = compare_instruction.GetOp1();
                shinst.common.compare_op.y = compare_instruction.GetOp2();
            }

            shinst.common.src1 = SourceRegister::FromTypeAndIndex(src1.GetType(), src1.GetIndex());
            shinst.common.src2 = SourceRegister::FromTypeAndIndex(src2.GetType(), src2.GetIndex());
            input_mask_src1 = src1.mask;
            input_mask_src2 = src2.mask;

            shinst.common.address_register_index = src1.relative_address_source;

            if (input_mask_src1.num_components != 2 || input_mask_src2.num_components != 2) {
                throw "Arguments need to have exactly two active components!"
                      + std::string("src1: ") + std::to_string(input_mask_src1.num_components) + " components, "
                      + std::string("src2: ") + std::to_string(input_mask_src2.num_components) + " components)";
            }

            // Build swizzle patterns
            SourceSwizzlerMask mask_src1;
            SourceSwizzlerMask mask_src2;
            mask_src1 = SourceSwizzlerMask::Expand(input_mask_src1);
            mask_src2 = SourceSwizzlerMask::Expand(input_mask_src2);
            shinst.common.operand_desc_id = FindOrAddSwizzlePattern(swizzle_patterns, mask_src1, mask_src2, src1.negate, src2.negate);

            instructions.push_back(shinst);
            ++program_write_offset;
        } else if (parser.ParseFlowControl(begin, input_code.end(), &statement_flow_control)) {
            auto abstract_opcode = statement_flow_control.GetOpCode();
            assert(abstract_opcode == OpCode::Id::GEN_IF   ||
                   abstract_opcode == OpCode::Id::GEN_JMP  ||
                   abstract_opcode == OpCode::Id::GEN_CALL ||
                   abstract_opcode == OpCode::Id::LOOP     ||
                   abstract_opcode == OpCode::Id::BREAKC);

            Instruction shinst;
            shinst.hex = 0;

            static const std::map<OpCode, OpCode> opcode_plain = {
                { OpCode::Id::GEN_CALL, OpCode::Id::CALL }
            };

            static const std::map<OpCode, OpCode> opcode_with_condition = {
                { OpCode::Id::BREAKC,   OpCode::Id::BREAKC }, // TODO: Make sure this isn't used outside of LOOPs
                { OpCode::Id::GEN_IF,   OpCode::Id::IFC },
                { OpCode::Id::GEN_JMP,  OpCode::Id::JMPC },
                { OpCode::Id::GEN_CALL, OpCode::Id::CALLC }
            };

            static const std::map<OpCode, OpCode> opcode_with_bool_uniform = {
                { OpCode::Id::GEN_IF,   OpCode::Id::IFU },
                { OpCode::Id::GEN_JMP,  OpCode::Id::JMPU },
                { OpCode::Id::GEN_CALL, OpCode::Id::CALLU }
            };

            static const std::map<OpCode, OpCode> opcode_with_int_uniform = {
                { OpCode::Id::LOOP,     OpCode::Id::LOOP }
            };

            if (statement_flow_control.HasCondition()) {
                const auto& condition = statement_flow_control.GetCondition();
                Atomic condition_variable = LookupIdentifier(condition.GetFirstInput().GetIdentifier());
                if (condition_variable.GetType() == RegisterType::ConditionalCode) {
                    // TODO: Make sure swizzle mask is either not set or x or y or xy
                    if (condition.GetFirstInput().HasSwizzleMask())
                        condition_variable.mask = MergeSwizzleMasks(condition_variable.mask, condition.GetFirstInput().GetSwizzleMask());

                    auto opcode = opcode_with_condition.find(abstract_opcode);
                    if (opcode == opcode_with_condition.end())
                        throw "May not pass a conditional code register to this instruction";

                    Instruction::FlowControlType::Op op;
                    bool negate_flags[2] = {};

                    if (condition.GetConditionOp() == Instruction::FlowControlType::JustX) {
                        if (condition_variable.mask.num_components == 1) {
                            if (condition_variable.mask.components[0] == InputSwizzlerMask::y) {
                                op = Instruction::FlowControlType::JustY;
                                negate_flags[1] = condition.GetFirstInput().GetInvertFlag();
                            } else {
                                op = Instruction::FlowControlType::JustX;
                                negate_flags[0] = condition.GetFirstInput().GetInvertFlag();
                            }
                        } else if (condition_variable.mask.num_components == 2) {
                            op = Instruction::FlowControlType::And;
                            negate_flags[0] = negate_flags[1] = condition.GetFirstInput().GetInvertFlag();
                        } else {
                            throw "May only involve the x and y components in conditions";
                        }
                    } else {
                        Atomic second_condition_variable = LookupIdentifier(condition.GetSecondInput().GetIdentifier());
                        if (second_condition_variable.GetType() != RegisterType::ConditionalCode)
                            throw "Combining conditions via && and || only works for conditions based on two conditional codes";

                        if (condition.GetSecondInput().HasSwizzleMask())
                            second_condition_variable.mask = MergeSwizzleMasks(second_condition_variable.mask, condition.GetSecondInput().GetSwizzleMask());

                        if (condition_variable.mask.num_components != 1 || second_condition_variable.mask.num_components != 1)
                            throw "Only one-component expressions can be combined via && and ||";

                        if (condition_variable.mask.components[0] == second_condition_variable.mask.components[0])
                            throw "Different conditional code components need to be used when combining conditions";

                        // Move x component to the first slot
                        if (condition_variable.mask.components[0] == InputSwizzlerMask::y) {
                            negate_flags[0] = condition.GetSecondInput().GetInvertFlag();
                            negate_flags[1] = condition.GetFirstInput().GetInvertFlag();
                        } else {
                            negate_flags[0] = condition.GetFirstInput().GetInvertFlag();
                            negate_flags[1] = condition.GetSecondInput().GetInvertFlag();
                        }

                        op = condition.GetConditionOp();
                    }

                    shinst.opcode = opcode->second;
                    shinst.flow_control.refx = !negate_flags[0];
                    shinst.flow_control.refy = !negate_flags[1];
                    shinst.flow_control.op = op;

                } else if (condition_variable.GetType() == RegisterType::BoolUniform) {
                    // TODO: Make sure swizzle mask is not set

                    if (condition.GetConditionOp() != Instruction::FlowControlType::JustX)
                        throw "May not combine conditions when branching on bool uniforms";

                    if (condition.GetFirstInput().GetInvertFlag())
                        throw "Negation cannot be used with boolean uniforms";

                    auto opcode = opcode_with_bool_uniform.find(abstract_opcode);
                    if (opcode == opcode_with_bool_uniform.end())
                        throw "May not pass a bool uniform register to this instruction";
                    shinst.opcode = opcode->second;

                    shinst.flow_control.bool_uniform_id = condition_variable.GetIndex();

                } else if (condition_variable.GetType() == RegisterType::IntUniform) {
                    // TODO: Make sure swizzle mask is not set

                    if (condition.GetConditionOp() != Instruction::FlowControlType::JustX)
                        throw "May not combine conditions when branching on bool uniforms";

                    if (condition.GetFirstInput().GetInvertFlag())
                        throw "Negation cannot be used with integer uniforms";

                    auto opcode = opcode_with_int_uniform.find(abstract_opcode);
                    if (opcode == opcode_with_int_uniform.end())
                        throw "May not pass an integer uniform register to this instruction";
                    shinst.opcode = opcode->second;

                    shinst.flow_control.int_uniform_id = condition_variable.GetIndex();

                } else {
                    throw "Unexpected register type passed as condition (must be conditional code or boolean uniform register)";
                }
            } else {
                switch (abstract_opcode) {
                case OpCode::Id::BREAKC:
                    throw "No condition passed for break-instruction";
                    break;

                case OpCode::Id::GEN_IF:
                    throw "No condition passed for if-instruction";
                    break;

                case OpCode::Id::LOOP:
                    throw "No condition passed for loop-instruction";
                    break;

                case OpCode::Id::GEN_JMP:
                    throw "No condition passed for jmp-instruction";
                    break;

                case OpCode::Id::GEN_CALL:
                    shinst.opcode = OpCode::Id::CALL;
                    break;

                default:
                    shinst.opcode = abstract_opcode;
                }
            }

            if (statement_flow_control.HasReturnLabel()) {

                if (abstract_opcode == OpCode::Id::GEN_JMP)
                    throw "Cannot specify return labels with jmp. Use call instead if you need automatic function returning.";

                assert(abstract_opcode == OpCode::Id::GEN_CALL);

                unsigned target_address = LookupLableAddress(statement_flow_control.GetTargetLabel());
                unsigned return_address = LookupLableAddress(statement_flow_control.GetReturnLabel());

                if (return_address <= target_address)
                    throw "Return address must be strictly bigger than target address";

                shinst.flow_control.num_instructions = return_address - target_address;
            } else {
                if (abstract_opcode == OpCode::Id::GEN_CALL) {
                    throw "Must specify a return label for call. Use jmp instead if you don't need automatic function returning.";
                }
            }

            if (abstract_opcode == OpCode::Id::GEN_IF) {
                call_stack.emplace(CallStackElement{abstract_opcode, (unsigned)instructions.size(), (unsigned)-1, (unsigned)-1});
            } else if (abstract_opcode == OpCode::Id::LOOP) {
                call_stack.emplace(CallStackElement{abstract_opcode, (unsigned)instructions.size(), (unsigned)-1, (unsigned)-1});
            } else if (abstract_opcode == OpCode::Id::BREAKC) {
                // Do nothing
            } else {
                shinst.flow_control.dest_offset = LookupLableAddress(statement_flow_control.GetTargetLabel());
            }

            instructions.push_back(shinst);
            ++program_write_offset;
        } else if (parser.ParseDeclaration(begin, input_code.end(), &statement_declaration)) {
            auto& var = statement_declaration;

            // TODO: Support not specifying any uniform name
            std::string idname = var.alias_name;

            Identifier id = var.identifier_start;
            Identifier end_id = (var.identifier_end) ? *var.identifier_end : id;
            bool aliases_range = (id != end_id);
            Atomic ret = LookupIdentifier(id);

            bool has_swizzle_mask = static_cast<bool>(var.swizzle_mask);
            ret.mask = has_swizzle_mask ? *var.swizzle_mask : InputSwizzlerMask::FullMask();

            std::vector<float>& values = var.extra.constant_value;
            auto output_semantic = var.extra.output_semantic;

            if (std::find(symbol_table.begin(), symbol_table.end(), idname) != symbol_table.end())
                throw "Symbol name \"" + idname + "\" already defined!";

            if (ret.GetType() != LookupIdentifier(end_id).GetType())
                throw "May not specify different register types for alias range limits";

            if (has_swizzle_mask && aliases_range)
                throw "May not use swizzle masks when aliasing register ranges";

            if (values.size()) {
                if (aliases_range)
                    throw "May not alias a register range when declaring constants";

                if (has_swizzle_mask)
                    throw "May not specify a swizzle mask when declaring constants";

                if (ret.GetType() != RegisterType::FloatUniform)
                    throw "Assigning constants to non-float uniforms is currently unsupported";

                if (values.size() != 4) {
                    std::stringstream ss;
                    ss <<"Must specify all register components when assigning constants (expected 4, got " << values.size() << ")";
                    throw ss.str();
                }

                // TODO: Support non-float constants

                ConstantInfo constant;
                constant.full_first_word = 0;
                constant.value_hex[0] = 0;
                constant.value_hex[1] = 0;
                constant.value_hex[2] = 0;
                constant.value_hex[3] = 0;
                constant.type = ConstantInfo::Float;
                constant.regid = LookupIdentifier(id).GetIndex();

                constant.f.x = to_float24(values[0]);
                constant.f.y = to_float24(values[1]);
                constant.f.z = to_float24(values[2]);
                constant.f.w = to_float24(values[3]);

                constant_table.push_back(constant);

            } else if (output_semantic) {
                if (ret.GetType() != RegisterType::Output)
                    throw "May not assign semantics to non-output registers.";

                if (aliases_range)
                    throw "May not alias a register range when aliasing output registers";

                // TODO: Make sure the declared output actually gets set (otherwise the GPU freezes)

                OutputRegisterInfo output;
                output.hex = 0;
                output.type = *output_semantic;
                output.id = LookupIdentifier(id).GetIndex();
                output.component_mask = 0;

                for (int i = 0; i < ret.mask.num_components; ++i) {
                    auto comp = ret.mask.components[i];
                    uint32_t added_mask = ((comp == InputSwizzlerMask::x) ? 1 :
                                           (comp == InputSwizzlerMask::y) ? 2 :
                                           (comp == InputSwizzlerMask::z) ? 4 :
                                           (comp == InputSwizzlerMask::w) ? 8 : 0);
                    if (output.component_mask >= added_mask)
                        throw "Invalid swizzle mask (may only mask away components for destination operands)";

                    output.component_mask = output.component_mask | added_mask;
                }
                output_table.push_back(output);

            } else {
                // plain uniform

                static auto ToCustomSpace = [](const Atomic& reg) -> int {
                    switch (reg.GetType()) {
                    case RegisterType::Input: return reg.GetIndex();
                    case RegisterType::FloatUniform: return reg.GetIndex() + 0x10;
                    case RegisterType::IntUniform: return reg.GetIndex() + 0x70;
                    case RegisterType::BoolUniform: return reg.GetIndex() + 0x78;
                    case RegisterType::Output: throw "Must specify output semantic for output register";

                    case RegisterType::Temporary:
                    default:
                        throw "May not assign aliases for the given register type";
                    }
                };

                UniformInfo uniform;
                uniform.basic.symbol_offset = [&]() { size_t ret = 0; for (auto& s : symbol_table) { ret +=s.length()+1; } return ret;}();
                uniform.basic.reg_start = ToCustomSpace(LookupIdentifier(id));
                uniform.basic.reg_end = ToCustomSpace(LookupIdentifier(end_id));
                uniform_table.push_back(uniform);

                symbol_table.push_back(idname);
            }

            identifier_table.insert({idname, ret});
        } else if (begin != input_code.end()) {
            // TODO: Actually, this should be a hint about invalid intruction formats, but on Windows even EOF triggers this for some reason.
            std::cerr << "Warning: Unknown instruction format, treating like EOF..." << std::endl;
            break;
        }
    }

    if (!call_stack.empty())
        throw "Not all IF/LOOP bodies are closed. Did you forget an ENDIF or ENDLOOP?";

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

    dvle.output_register_table_offset = write_offset - dvlb.dvle_offset;
    dvle.output_register_table_size = output_table.size();
    for (const auto& output : output_table) {
        QueueForWriting((uint8_t*)&output, sizeof(output));
    }

    dvle.constant_table_offset = write_offset - dvlb.dvle_offset;
    dvle.constant_table_size = constant_table.size();
    for (const auto& constant : constant_table) {
        QueueForWriting((uint8_t*)&constant, sizeof(constant));
    }


    // TODO: UniformTable spans more than the written data.. fix this design issue :/
    // TODO: Is this TODO still valid?
    dvle.uniform_table_offset = write_offset - dvlb.dvle_offset;
    dvle.uniform_table_size = uniform_table.size();
    for (const auto& uniform : uniform_table) {
        QueueForWriting((uint8_t*)&reinterpret_cast<const uint64_t&>(uniform.basic), sizeof(uint64_t));
    }


    dvle.main_offset_words = main_offset;

    dvle.label_table_offset = write_offset - dvlb.dvle_offset;
    dvle.label_table_size = label_table.size();
    std::vector<LabelInfo> final_label_table;
    final_label_table.reserve(label_table.size());
    for (auto& label : label_table) {
        LabelInfo info;
        memset(&info, 0, sizeof(info));
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

    } catch (const char* err) {
        std::cerr << input_filename << ":" << code_line << ": error: " << err << std::endl;
        size_t start_pos = std::distance(input_code.begin(), preparse_begin);
        std::cerr << "\t" << input_code.substr(start_pos, input_code.find('\n', start_pos) - start_pos) << std::endl;
    }
    catch (const std::string& err) {
        std::cerr << input_filename << ":" << code_line << ": error: " << err << std::endl;
        size_t start_pos = std::distance(input_code.begin(), preparse_begin);
        std::cerr << "\t" << input_code.substr(start_pos, input_code.find('\n', start_pos) - start_pos) << std::endl;
        return 1;
    }

    // Write data to file
    std::ofstream file(output_filename, std::ios_base::out | std::ios_base::binary);

    if (!file) {
        std::cerr << "Could not open output file " << output_filename << std::endl;
        return 1;
    }

    for (auto& chunk : writing_queue) {
        file.write((char*)chunk.pointer, chunk.size);
    }

    return 0;
}
