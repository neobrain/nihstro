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

// Token sequence definitions
using RegisterWithIndex = boost::fusion::vector<Instruction::RegisterType, // input/output/uniform...
                                                int,                       // index
                                                boost::optional<int>       // swizzle mask
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

        swizzlers.add( "x",   1 )
                     ( "y",   2 )
                     ( "z",   4 )
                     ( "w",   8 )
                     ( "xy",  1 | 2 )
                     ( "xz",  1 | 4 )
                     ( "xw",  1 | 8 )
                     ( "yz",  2 | 4 )
                     ( "yw",  2 | 8 )
                     ( "zw",  4 | 8 )
                     ( "xyz", 1 | 2 | 4 )
                     ( "xyw", 1 | 2 | 8 )
                     ( "yzw", 2 | 4 | 8 )
                     ( "xyz", 1 | 2 | 4 | 8 );

        // Setup rules
        identifier = qi::lexeme[+(qi::char_("a-zA-Z_")) >> -+qi::char_("0-9")];

        auto label = identifier >> qi::lit(':');

		register_with_index = qi::lexeme[(register_prefixes >> qi::int_) >> -('.' >> swizzlers)];
        auto instr = qi::no_case[qi::lexeme[opcodes]] >> (register_with_index % ',');

        start %= label | instr;
    }

    qi::symbols<char, Instruction::OpCode>       opcodes;
    qi::symbols<char, Instruction::RegisterType> register_prefixes;
    qi::symbols<char, int>                       swizzlers;

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
                    case Instruction::OpCodeType::CommonLong:
                    case Instruction::OpCodeType::CommonShort:
                    {
                        const int num_inputs = (shinst.opcode.GetInfo().type == Instruction::OpCodeType::CommonLong) ?
                                               2 : 1;
                        if (num_args < num_inputs + 1)
                            throw "Incorrect number of arguments. Expected " + std::to_string(num_inputs + 1) + ", got " + std::to_string(num_args);

                        int src2_mask = 0;
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
                            src2_mask = (at_c<2>(regs_with_index[2]) == boost::none) ? 0xF : *at_c<2>(regs_with_index[2]);
                        }
                        shinst.common.dest.InitializeFromTypeAndIndex(at_c<0>(regs_with_index[0]), at_c<1>(regs_with_index[0]));
                        shinst.common.src1.InitializeFromTypeAndIndex(at_c<0>(regs_with_index[1]), at_c<1>(regs_with_index[1]));

                        // If no swizzler have been specified, use .xyzw
                        int dest_mask_inverse = (at_c<2>(regs_with_index[0]) == boost::none) ? 0xF : *at_c<2>(regs_with_index[0]);
                        int src1_mask         = (at_c<2>(regs_with_index[1]) == boost::none) ? 0xF : *at_c<2>(regs_with_index[1]);

                        auto MaskBitSet = [](int i, int mask) {
                            return (mask >> i) & 1;
                        };

                        auto GetNumComponentsFromMask = [&MaskBitSet](int mask) {
                            return MaskBitSet(0, mask) + MaskBitSet(1, mask)
                                 + MaskBitSet(2, mask) + MaskBitSet(3, mask);
                        };

                        if (GetNumComponentsFromMask(dest_mask_inverse) != GetNumComponentsFromMask(src1_mask) ||
                            (num_inputs > 1 && GetNumComponentsFromMask(src1_mask) != GetNumComponentsFromMask(src2_mask))) {
                            throw "Input registers need to use the same number of components as the output register!"
                                  + std::string("(dest: ") + std::to_string(GetNumComponentsFromMask(dest_mask_inverse)) + " components, "
                                  + std::string("src1: ") + std::to_string(GetNumComponentsFromMask(src1_mask)) + " components"
                                  + ((num_inputs > 1) ? std::string(", src2: ") + std::to_string(GetNumComponentsFromMask(src2_mask)) + " components)"
                                                      : std::string(")"));
                        }

                        auto SourceMaskToSelectorVector = [&MaskBitSet](int mask) {
                            std::vector<SwizzlePattern::Selector> ret;

                            for (int i = 0; i < 4; ++i)
                                if (MaskBitSet(i, mask))
                                    ret.push_back(static_cast<SwizzlePattern::Selector>(i));

                            return ret;
                        };

                        SwizzlePattern swizzle_pattern;
                        swizzle_pattern.hex = 0;
                        int active_component = 0;
                        for (int i = 0; i < 4; ++i) {
                            if (dest_mask_inverse & (1 << i)) {
                                swizzle_pattern.SetDestComponentEnabled(i, true);

                                auto GetSrcComponentIndex = [&MaskBitSet, &active_component](int mask) {
                                                                int comp = active_component;
                                                                for (int i = 0; i < 4; ++i)
                                                                    if (MaskBitSet(i, mask))
                                                                        if (comp-- == 0) // post-increment intended
                                                                            return i;
                                                            };

                                swizzle_pattern.SetSelectorSrc1(i, SourceMaskToSelectorVector(src1_mask)[active_component]);
                                if (num_inputs > 1)
                                    swizzle_pattern.SetSelectorSrc2(i, SourceMaskToSelectorVector(src2_mask)[active_component]);

                                active_component++;
                            }
                        }
                        // TODO: support other fields in SwizzlePattern!
                        auto it = std::find_if(swizzle_patterns.begin(), swizzle_patterns.end(),
                                               [&swizzle_pattern](const SwizzlePattern& val) { return val.hex == swizzle_pattern.hex; });
                        if (it == swizzle_patterns.end()) {
                            swizzle_patterns.push_back(swizzle_pattern);
                            it = swizzle_patterns.end() - 1;

                            if (swizzle_patterns.size() > 127)
                                throw "Maximum number of swizzle patterns 127 has been exhausted";
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
