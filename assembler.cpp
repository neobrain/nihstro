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

// Token definitions
struct TokenRegister : boost::fusion::vector<Instruction::RegisterType,         // input/output/uniform...
                                             int,                               // index
                                             boost::optional<InputSwizzlerMask> // swizzle mask
                                            > {

    const Instruction::RegisterType& GetType() const {
        return boost::fusion::at_c<0>(*this);
    }

    const int& GetIndex() const {
        return boost::fusion::at_c<1>(*this);
    }

    // Returns .xyzw if input swizzler mask is not set
    const InputSwizzlerMask GetInputSwizzlerMask() const {
        if (boost::fusion::at_c<2>(*this) == boost::none)
            return InputSwizzlerMask::FullMask();
        else
            return boost::fusion::at_c<2>(*this).get();
    }
};

using TokenIdentifier = std::string;
using TokenConstant = int;

struct Token : boost::variant<Instruction::OpCode,
                              TokenRegister,
                              TokenIdentifier,
                              TokenConstant> {
    enum Type {
        OpCode     = 0,
        Register   = 1,
        Identifier = 2,
        Constant   = 3,
    };

    using BaseType = boost::variant<Instruction::OpCode,
                                    TokenRegister,
                                    TokenIdentifier,
                                    TokenConstant
                                   >;
    Token() = default;
    Token(const BaseType& var) : BaseType(var) {}

    bool HasType(Type type) const {
        return which() == type;
    }

    // NOTE: Caller is responsible for making sure the requested type is active!
    const Instruction::OpCode& GetOpCode() const {
        return boost::get<Instruction::OpCode>(*this);
    }

    const TokenRegister& GetRegister() const {
        return boost::get<TokenRegister>(*this);
    }

    const TokenIdentifier& GetIdentifier() const {
        return boost::get<TokenIdentifier>(*this);
    }

    const TokenConstant& GetConstant() const {
        return boost::get<TokenConstant>(*this);
    }
};

using TokenSequenceLabel = std::string;

struct TokenSequenceInstruction : std::vector<Token> {

};

using TokenSequence = boost::variant<TokenSequenceLabel,
                                     TokenSequenceInstruction>;

template<typename Iterator>
struct AssemblyParser : qi::grammar<Iterator, TokenSequence(), AssemblySkipper<Iterator>> {
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

        register_rule = qi::lexeme[(register_prefixes >> qi::int_) >> -('.' >> swizzlers)];
        opcode = qi::no_case[qi::lexeme[opcodes]];

        token = opcode | register_rule | identifier | qi::int_;

        instr = token >> (token % ','); // e.g. "add o1, t2, t5"

        start %= label | instr;
    }

    qi::symbols<char, Instruction::OpCode>        opcodes;
    qi::symbols<char, Instruction::RegisterType>  register_prefixes;
    qi::symbols<char, InputSwizzlerMask>          swizzlers;

    qi::rule<Iterator, TokenSequenceInstruction(),Skipper> instr;
    qi::rule<Iterator, Instruction::OpCode(),     Skipper> opcode;
    qi::rule<Iterator, std::string(),             Skipper> identifier;
    qi::rule<Iterator, TokenRegister(),           Skipper> register_rule;
    qi::rule<Iterator, TokenSequence(),           Skipper> start;
    qi::rule<Iterator, Token(),                   Skipper> token;
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

    TokenSequence token_sequence;
    while (phrase_parse(begin, input_code.end(), parser, skipper, token_sequence)) {

        if (token_sequence.which() == 0) {
            std::string label_symbol = boost::get<TokenSequenceLabel>(token_sequence);

            auto it = std::find(symbol_table.begin(), symbol_table.end(), label_symbol);
            if (it != symbol_table.end())
                throw "Label \"" + label_symbol + "\" already defined in symbol table";

            symbol_table.push_back(label_symbol);
            uint32_t symbol_table_index = symbol_table.size() - 1;

            CustomLabelInfo label_info = { program_write_offset, symbol_table_index };
            label_table.push_back(label_info);
        } else if (token_sequence.which() == 1) {
            auto& instr = boost::get<TokenSequenceInstruction>(token_sequence);

            if (!instr[0].HasType(Token::OpCode))
                throw "Instruction statements need to start with an opcode";

            Instruction shinst;
            shinst.hex = 0;
            shinst.opcode.Assign(instr[0].GetOpCode());
            int num_args = instr.size() - 1;
            switch (shinst.opcode.GetInfo().type) {
                case Instruction::OpCodeType::Arithmetic:
                {
                    const int num_inputs = shinst.opcode.GetInfo().num_arguments - 1;
                    if (num_args != num_inputs + 1)
                        throw "Incorrect number of arguments. Expected " + std::to_string(num_inputs + 1) + ", got " + std::to_string(num_args);

                    if (!instr[1].HasType(Token::Register))
                        throw "Unexpected token in arithmetic instruction statement: Expected register identifier as destination argument";

                    if (!instr[2].HasType(Token::Register))
                        throw "Unexpected token in arithmetic instruction statement: Expected register identifier as first source argument";

                    // If no swizzler have been specified, use .xyzw - compile errors triggered by this are intended! (accessing subvectors should be done explicitly)
                    InputSwizzlerMask input_dest_mask = instr[1].GetRegister().GetInputSwizzlerMask();
                    InputSwizzlerMask input_mask_src1;
                    InputSwizzlerMask input_mask_src2;
                    if (num_inputs > 1) {
                        if (!instr[3].HasType(Token::Register))
                            throw "Unexpected token in arithmetic instruction statement: Expected register identifier as second source argument";

                        if (instr[2].GetRegister().GetType() == Instruction::FloatUniform &&
                            instr[3].GetRegister().GetType() == Instruction::FloatUniform) {
                            throw "Not more than one input register may be a floating point uniform";
                        }

                        // If second argument is a floating point register, swap it to first place
                        if (instr[3].GetRegister().GetType() == Instruction::FloatUniform) {
                            boost::swap(instr[2], instr[3]);
                        }

                        shinst.common.src2.InitializeFromTypeAndIndex(instr[3].GetRegister().GetType(), instr[3].GetRegister().GetIndex());
                        input_mask_src2 = instr[3].GetRegister().GetInputSwizzlerMask();
                    }
                    input_mask_src1 = instr[2].GetRegister().GetInputSwizzlerMask();

                    shinst.common.dest.InitializeFromTypeAndIndex(instr[1].GetRegister().GetType(), instr[1].GetRegister().GetIndex());
                    shinst.common.src1.InitializeFromTypeAndIndex(instr[2].GetRegister().GetType(), instr[2].GetRegister().GetIndex());

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
    }

    // Error out if we didn't parse the full file
    if (begin != input_code.end()) {
        throw "Invalid token found: " + input_code.substr(begin - input_code.begin());
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
        std::cout << "Error: " << err << std::endl;
        return 1;
    } catch(const char* err) {
        std::cout << "Error: " << err << std::endl;
        return 1;
    }

    return 0;
}
