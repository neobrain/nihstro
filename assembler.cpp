#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/spirit/include/qi.hpp>

#include "shader_binary.h"
#include "shader_bytecode.h"

using namespace boost::spirit;

template<typename Iterator>
struct AssemblySkipper : public qi::grammar<Iterator> {

    AssemblySkipper() : AssemblySkipper::base_type(skip) {

        comments = qi::char_("//") >> *(qi::char_ - qi::eol) >> qi::eol;

        skip = +(comments | ascii::space);
    }

    qi::rule<Iterator> comments;
    qi::rule<Iterator> skip;
};


struct InputSwizzlerMask {
    int num_components;

    enum Component : uint8_t {
        x = 0,
        y = 1,
        z = 2,
        w = 3,
    };
    Component components[4];

    static InputSwizzlerMask FullMask() {
        return { 4, {x,y,z,w} };
    }

    bool operator == (const InputSwizzlerMask& oth) const {
        return 0 == memcmp(this, &oth, sizeof(InputSwizzlerMask));
    }
};

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

// Token sequence definitions
using RegisterWithIndex = boost::fusion::vector<Instruction::RegisterType,         // input/output/uniform...
                                                int,                               // index
                                                boost::optional<InputSwizzlerMask> // swizzle mask
                                               >;

using SequenceLabel = std::string;

// An opcode + an unspecified number of registers
using SequenceInstruction = boost::fusion::vector<Instruction::OpCode, std::vector<RegisterWithIndex>>;

using InstructionVariant = boost::variant<SequenceLabel,
                                   SequenceInstruction>;

template<typename Iterator>
struct AssemblyParser : qi::grammar<Iterator, InstructionVariant(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    AssemblyParser() : AssemblyParser::base_type(start) {
        // Setup symbol table
        opcodes.add
                   ( "add",   Instruction::OpCode::ADD   )
                   ( "dp3",   Instruction::OpCode::DP3   )
                   ( "dp4",   Instruction::OpCode::DP4   )
                   ( "mul",   Instruction::OpCode::MUL   )
                   ( "max",   Instruction::OpCode::MAX   )
                   ( "min",   Instruction::OpCode::MIN   )
                   ( "rcp",   Instruction::OpCode::RCP   )
                   ( "rsq",   Instruction::OpCode::RSQ   )
                   ( "mov",   Instruction::OpCode::MOV   )
                   ( "ret",   Instruction::OpCode::RET   )
                   ( "flush", Instruction::OpCode::FLUSH )
                   ( "call",  Instruction::OpCode::CALL  )
                   ( "cmp",   Instruction::OpCode::CMP   );

        register_prefixes.add( "i", Instruction::Input)
                             ( "o", Instruction::Output)
                             ( "t", Instruction::Temporary)
                             ( "f", Instruction::FloatUniform);

        // TODO: Might want to change to only have "x", "y", "z" and "w"
        swizzlers.add( "x",    {1, {InputSwizzlerMask::x}} )
                     ( "y",    {1, {InputSwizzlerMask::y}} )
                     ( "z",    {1, {InputSwizzlerMask::z}} )
                     ( "w",    {1, {InputSwizzlerMask::w}} )
                     ( "xy",   {2, {InputSwizzlerMask::x,InputSwizzlerMask::y}} )
                     ( "xz",   {2, {InputSwizzlerMask::x,InputSwizzlerMask::z}} )
                     ( "xw",   {2, {InputSwizzlerMask::x,InputSwizzlerMask::w}} )
                     ( "yz",   {2, {InputSwizzlerMask::y,InputSwizzlerMask::z}} )
                     ( "yw",   {2, {InputSwizzlerMask::y,InputSwizzlerMask::w}} )
                     ( "zw",   {2, {InputSwizzlerMask::z,InputSwizzlerMask::w}} )
                     ( "xyz",  {3, {InputSwizzlerMask::x,InputSwizzlerMask::y,InputSwizzlerMask::z}} )
                     ( "xyw",  {3, {InputSwizzlerMask::x,InputSwizzlerMask::y,InputSwizzlerMask::w}} )
                     ( "xzw",  {3, {InputSwizzlerMask::x,InputSwizzlerMask::z,InputSwizzlerMask::w}} )
                     ( "yzw",  {3, {InputSwizzlerMask::y,InputSwizzlerMask::z,InputSwizzlerMask::w}} )
                     ( "xyzw", {4, {InputSwizzlerMask::x,InputSwizzlerMask::y,InputSwizzlerMask::z,InputSwizzlerMask::w}} );

        // Setup rules
        identifier = qi::lexeme[+(qi::char_("a-zA-Z_")) >> -+qi::char_("0-9")];

        auto label = identifier >> qi::lit(':');

        register_with_index = qi::lexeme[(register_prefixes >> qi::int_) >> -('.' >> swizzlers)];
        auto instr = qi::no_case[qi::lexeme[opcodes]] >> (register_with_index % ',');

        start %= label | instr;
    }

    qi::symbols<char, Instruction::OpCode>       opcodes;
    qi::symbols<char, Instruction::RegisterType> register_prefixes;
    qi::symbols<char, InputSwizzlerMask>         swizzlers;

    qi::rule<Iterator, std::string(),        Skipper> identifier;
    qi::rule<Iterator, RegisterWithIndex(),  Skipper> register_with_index;
    qi::rule<Iterator, InstructionVariant(), Skipper> start;
};

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cout << "No output filename given" << std::endl;
        return 1;
    }

    if (argc < 3) {
        std::cout << "No input filenames given" << std::endl;
        return 1;
    }

    try {

    std::string output_filename = argv[1];
    std::string input_filename = argv[2];

    AssemblyParser<std::string::iterator> parser;
    AssemblySkipper<std::string::iterator> skipper;

    std::ifstream input_file(input_filename);
    std::string input_code;
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

    std::string::iterator begin = input_code.begin();

    std::vector<Instruction> instructions;
    std::vector<SwizzlePattern> swizzle_patterns;

    struct CustomLabelInfo {
        uint32_t program_offset;
        uint32_t symbol_table_index;
    };

    std::vector<CustomLabelInfo> label_table;

    std::vector<std::string> symbol_table;

    uint32_t program_write_offset = 0;

    do {
        InstructionVariant instr;

        bool r = phrase_parse(begin, input_code.end(),
                              parser, skipper, instr);

        if (r) {
            using boost::fusion::at_c;

            if (instr.which() == 0) {
                std::string label_symbol = boost::get<SequenceLabel>(instr);

                auto it = std::find(symbol_table.begin(), symbol_table.end(), label_symbol);
                if (it != symbol_table.end())
                    throw "Label \"" + label_symbol + "\" already defined in symbol table";

                symbol_table.push_back(label_symbol);
                uint32_t symbol_table_index = symbol_table.size() - 1;

                CustomLabelInfo label_info = { program_write_offset, symbol_table_index };
                label_table.push_back(label_info);
            } else if (instr.which() == 1) {
                const auto& seqinstr = boost::get<SequenceInstruction>(instr);

                Instruction shinst;
                shinst.hex = 0;
                shinst.opcode.Assign(at_c<0>(seqinstr));
                std::vector<RegisterWithIndex> regs_with_index = at_c<1>(seqinstr);
                int num_args = regs_with_index.size();;
                switch (shinst.opcode.GetInfo().type) {
                    case Instruction::OpCodeType::Arithmetic:
                    {
                        const int num_inputs = shinst.opcode.GetInfo().num_arguments - 1;
                        if (num_args < num_inputs + 1)
                            throw "Incorrect number of arguments. Expected " + std::to_string(num_inputs + 1) + ", got " + std::to_string(num_args);

                        // If no swizzler have been specified, use .xyzw - compile errors triggered by this are intended! (accessing subvectors should be done explicitly)
                        InputSwizzlerMask input_dest_mask = (at_c<2>(regs_with_index[0]) == boost::none)
                                                            ? InputSwizzlerMask::FullMask()
                                                            : *at_c<2>(regs_with_index[0]);
                        InputSwizzlerMask input_mask_src1;
                        InputSwizzlerMask input_mask_src2;
                        if (num_inputs > 1) {
                            if (at_c<0>(regs_with_index[1]) == Instruction::FloatUniform &&
                                at_c<0>(regs_with_index[2]) == Instruction::FloatUniform) {
                                throw "Not more than one input register may be a floating point uniform";
                            }

                            // If second argument is a floating point register, swap it to first place
                            if (at_c<0>(regs_with_index[2]) == Instruction::FloatUniform) {
                                boost::swap(regs_with_index[1], regs_with_index[2]);
                            }

                            shinst.common.src2.InitializeFromTypeAndIndex(at_c<0>(regs_with_index[2]), at_c<1>(regs_with_index[2]));
                            input_mask_src2 = (at_c<2>(regs_with_index[2]) == boost::none) ? InputSwizzlerMask::FullMask() : *at_c<2>(regs_with_index[2]);
                        }
                        input_mask_src1 = (at_c<2>(regs_with_index[1]) == boost::none) ? InputSwizzlerMask::FullMask() : *at_c<2>(regs_with_index[1]);

                        shinst.common.dest.InitializeFromTypeAndIndex(at_c<0>(regs_with_index[0]), at_c<1>(regs_with_index[0]));
                        shinst.common.src1.InitializeFromTypeAndIndex(at_c<0>(regs_with_index[1]), at_c<1>(regs_with_index[1]));

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

                        // TODO: support other fields in SwizzlePattern (e.g. negate_src1)
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
            }
        } else {
            throw "Invalid token found: " + std::to_string(begin - input_code.begin());
        }
    } while (begin != input_code.end());

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
        std::cout << "Error: " << err << std::endl;
        return 1;
    } catch(const char* err) {
        std::cout << "Error: " << err << std::endl;
        return 1;
    }

    return 0;
}
