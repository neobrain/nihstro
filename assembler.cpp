#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/spirit/include/qi.hpp>

#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>


#include "shader_binary.h"
#include "shader_bytecode.h"

using namespace boost::spirit;
namespace phoenix = boost::phoenix;

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

// Identifer index in identifier list
using Identifier = int;

// TODO: This is still quite the limited expression. Ideally we want to support nested expressions, too
struct Expression : boost::fusion::vector<Identifier, std::vector<InputSwizzlerMask>> {

};

using StatementLabel = std::string;

// TODO: Figure out why this cannot be a std::tuple...
struct StatementInstruction : boost::fusion::vector<Instruction::OpCode, std::vector<Expression>> {
    const Instruction::OpCode& GetOpCode() const {
        return boost::fusion::at_c<0>(*this);
    }

    const std::vector<Expression>& GetArguments() const {
        return boost::fusion::at_c<1>(*this);
    }
};


enum class RegisterNameType {
    OutputPos   = 0,
    Input       = 1,
    Constant    = 2
};

struct StatementRegisterName : boost::fusion::vector<RegisterNameType, std::string, Identifier> {

};

using Statement = boost::variant<StatementLabel,
                                 StatementInstruction,
                                 StatementRegisterName>;

struct ParserContext {
    // Maps known identifiers to an index to the controller's identifier list
    qi::symbols<char, Identifier> identifiers;
};

template<typename Iterator>
struct AssemblyParser : qi::grammar<Iterator, Statement(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    AssemblyParser(const ParserContext& context) : AssemblyParser::base_type(start) {

        // Setup symbol table
        opcodes[0].add
                   ( "ret",   Instruction::OpCode::RET   )
                   ( "flush", Instruction::OpCode::FLUSH );
        opcodes[1].add
                   ( "call",  Instruction::OpCode::CALL  );

        opcodes[2].add
                   ( "mov",   Instruction::OpCode::MOV   )
                   ( "rcp",   Instruction::OpCode::RCP   )
                   ( "rsq",   Instruction::OpCode::RSQ   );
        opcodes[3].add
                   ( "add",   Instruction::OpCode::ADD   )
                   ( "mul",   Instruction::OpCode::MUL   )
                   ( "dp3",   Instruction::OpCode::DP3   )
                   ( "dp4",   Instruction::OpCode::DP4   )
                   ( "max",   Instruction::OpCode::MAX   )
                   ( "min",   Instruction::OpCode::MIN   );
        opcodes[4].add
                   ( "cmp",   Instruction::OpCode::CMP   );

        register_prefixes.add( "i", Instruction::Input)
                             ( "o", Instruction::Output)
                             ( "t", Instruction::Temporary)
                             ( "f", Instruction::FloatUniform);

        // TODO: Might want to change to only have "x", "y", "z" and "w"
        // TODO: Add rgba/stq masks
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

        define_out_registers.add("out_pos", RegisterNameType::OutputPos);
        define_out_registers.add("alias", RegisterNameType::Input);
        define_out_registers.add("const", RegisterNameType::Constant);

        // Setup rules
        identifier = qi::lexeme[+(qi::char_("a-zA-Z_")) >> -+qi::char_("0-9")];

        auto label = identifier >> qi::lit(':');

		for (int i = 0; i < 5; ++i) {
            // Make sure that a mnemonic is always followed by a space
            opcode[i] = qi::no_case[qi::lexeme[opcodes[i] >> qi::omit[ascii::blank]]];
        }

        swizzle_mask = qi::lexeme[swizzlers];
        expression = qi::lexeme[context.identifiers] >> *(qi::lit('.') >> swizzle_mask);

        // e.g. "add o1, t2, t5"
        // TODO: Using > instead of >> would break the argument number detection, for whatever reason...
        //       It would match whole lines, but never read more than one argument. Strange.
		auto comma_rule = qi::lit(',');
		instr_extra_argument = comma_rule >> expression;
        not_comma = !comma_rule;
        // TODO: These don't work for reasons I don't understand. need to fix that somehow...
//        instr[0] = opcode[0] >> repeat(0)[expression] >> not_comma; // add repeat(0) so that it matches against tuple<.,vector<.>>
//        instr[1] = opcode[1] >> expression >> not_comma;
//        instr[2] = opcode[2] >> expression >> instr_extra_argument >> not_comma;
//        instr[3] = opcode[3] >> expression >> instr_extra_argument >> instr_extra_argument >> not_comma;
//        instr[4] = opcode[4] >> expression >> instr_extra_argument >> instr_extra_argument >> instr_extra_argument >> not_comma;
        instr[0] = opcode[0] >> (expression % qi::lit(',')) >> not_comma;
        instr[1] = opcode[1] >> (expression % qi::lit(',')) >> not_comma;
        instr[2] = opcode[2] >> (expression % qi::lit(',')) >> not_comma;
        instr[3] = opcode[3] >> (expression % qi::lit(',')) >> not_comma;
        instr[4] = opcode[4] >> (expression % qi::lit(',')) >> not_comma;

        defineoutreg = qi::lit('.') >> qi::lexeme[define_out_registers >> ascii::blank] >> identifier >> ',' >> qi::lexeme[context.identifiers];

        // TODO: Expect a newline at the end of things...
        start %= label | (instr[0] | instr[1] | instr[2] | instr[3] | instr[4]) | defineoutreg;

		// Error handling
        instr_extra_argument.name("additional argument");
        not_comma.name("not comma");
        expression.name("expression");
        swizzle_mask.name("swizzle mask");

        // TODO: Make these error messages more helpful...
        // _1: Iterator first
        // _2: Iterator last
        // _3: Iterator err_pos
        // _4: spirit::info const &what
		qi::on_error<qi::fail>
		(
			start
          , std::cout
                << phoenix::val("Error! Expected ")
                << _4                               // what failed?
                << phoenix::val(" here: \"")
                << phoenix::construct<std::string>(_1, _3) + "___" + phoenix::construct<std::string>(_3+1, _2)
                << phoenix::val("\"")
                << std::endl
		);
    }

    qi::symbols<char, Instruction::OpCode>        opcodes[5]; // indexed by number of arguments
    qi::symbols<char, Instruction::RegisterType>  register_prefixes;
    qi::symbols<char, InputSwizzlerMask>          swizzlers;
    qi::symbols<char, RegisterNameType>           define_out_registers;

    qi::rule<Iterator, StatementInstruction(),    Skipper> instr[5];
    qi::rule<Iterator, Instruction::OpCode(),     Skipper> opcode[5];
    qi::rule<Iterator, std::string(),             Skipper> identifier;
    qi::rule<Iterator, Statement(),               Skipper> start;
    qi::rule<Iterator, StatementRegisterName(),   Skipper> defineoutreg;
    qi::rule<Iterator, Expression(),              Skipper> instr_extra_argument;
    qi::rule<Iterator,                            Skipper> not_comma;

    qi::rule<Iterator, Expression(),              Skipper> expression;
    qi::rule<Iterator, InputSwizzlerMask(),       Skipper> swizzle_mask;
};

enum class RegisterSpace : int {
    Input        = 0,
    Temporary    = 0x10,
    FloatUniform = 0x20,
    Output       = 0x80,
};

// Smallest unit an expression evaluates to:
// Index to register + number of components + swizzle mask
// Labels are different.
struct Atomic {
    int register_index;
    InputSwizzlerMask mask;

    const Instruction::RegisterType GetType() const {
        if (register_index > (int)RegisterSpace::Output)
            return Instruction::Output;
        else if (register_index > (int)RegisterSpace::FloatUniform)
            return Instruction::FloatUniform;
        else if (register_index > (int)RegisterSpace::Temporary)
            return Instruction::Temporary;
        else if (register_index > (int)RegisterSpace::Input)
            return Instruction::Input;
    }

    int GetIndex() const {
        if (register_index > (int)RegisterSpace::Output)
            return register_index - (int)RegisterSpace::Output;
        else if (register_index > (int)RegisterSpace::FloatUniform)
            return register_index - (int)RegisterSpace::FloatUniform;
        else if (register_index > (int)RegisterSpace::Temporary)
            return register_index - (int)RegisterSpace::Temporary;
        else if (register_index > (int)RegisterSpace::Input)
            return register_index - (int)RegisterSpace::Input;
    }
};

// TODO: Support labels as identifiers...
std::vector<Atomic> identifiers;

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
    for (int i = 0; i < 0x90; ++i) {
        identifiers.push_back({i, InputSwizzlerMask::FullMask()});

        std::string name;
        if (i >= (int)RegisterSpace::Output)
            name = "o" + std::to_string(i - (int)RegisterSpace::Output);
        else if (i >= (int)RegisterSpace::FloatUniform)
            name = "f" + std::to_string(i - (int)RegisterSpace::FloatUniform);
        else if (i >= (int)RegisterSpace::Temporary)
            name = "t" + std::to_string(i - (int)RegisterSpace::Temporary);
        else if (i >= (int)RegisterSpace::Input)
            name = "i" + std::to_string(i - (int)RegisterSpace::Input);

        context.identifiers.add(name, i);
    }


    while (true) {
        AssemblyParser<std::string::iterator> parser(context);
        AssemblySkipper<std::string::iterator> skipper;
        Statement statement;

        preparse_begin = begin;
        if (false == phrase_parse(begin, input_code.end(), parser, skipper, statement))
            break;

        if (statement.which() == 0) {
            std::string label_symbol = boost::get<StatementLabel>(statement);

            auto it = std::find(symbol_table.begin(), symbol_table.end(), label_symbol);
            if (it != symbol_table.end())
                throw "Label \"" + label_symbol + "\" already defined in symbol table";

            symbol_table.push_back(label_symbol);
            uint32_t symbol_table_index = symbol_table.size() - 1;

            CustomLabelInfo label_info = { program_write_offset, symbol_table_index };
            label_table.push_back(label_info);
        } else if (statement.which() == 1) {
            auto& instr = boost::get<StatementInstruction>(statement);

            Instruction shinst;
            shinst.hex = 0;
            shinst.opcode.Assign(instr.GetOpCode());
            const std::vector<Expression>& args = instr.GetArguments();
            std::vector<Atomic> arguments;
            for (const auto& expr : args) {
                auto EvaluateExpression = [](const Expression& expr) {
                    Atomic ret = identifiers[boost::fusion::at_c<0>(expr)];

                    // Apply swizzle mask(s)
                    for (const auto& swizzle_mask : boost::fusion::at_c<1>(expr)) {
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
                    const int num_inputs = shinst.opcode.GetInfo().num_arguments - 1;
                    if (num_args != num_inputs + 1)
                        throw "Incorrect number of arguments. Expected " + std::to_string(num_inputs + 1) + ", got " + std::to_string(num_args);

                    auto AssertRegisterReadable = [](Instruction::RegisterType type) {
                        if (type != Instruction::Input && type != Instruction::Temporary &&
                            type != Instruction::FloatUniform)
                            throw "Specified register is not readable (only input, temporary and uniform registers are writeable)";
                    };
                    auto AssertRegisterWriteable = [](Instruction::RegisterType type) {
                        if (type != Instruction::Output && type != Instruction::Temporary)
                            throw "Specified register is not writeable (only output and temporary registers are writeable)";
                    };
                    AssertRegisterWriteable(arguments[0].GetType());
                    AssertRegisterReadable(arguments[1].GetType());

                    // If no swizzler have been specified, use .xyzw - compile errors triggered by this are intended! (accessing subvectors should be done explicitly)
                    InputSwizzlerMask input_dest_mask = arguments[0].mask;
                    InputSwizzlerMask input_mask_src1;
                    InputSwizzlerMask input_mask_src2;
                    if (num_inputs > 1) {
                        AssertRegisterReadable(arguments[2].GetType());

                        if (arguments[1].GetType() == Instruction::FloatUniform &&
                            arguments[2].GetType() == Instruction::FloatUniform) {
                            throw "Not more than one input register may be a floating point uniform";
                        }

                        // If second argument is a floating point register, swap it to first place
                        if (arguments[2].GetType() == Instruction::FloatUniform) {
                            boost::swap(arguments[1], arguments[2]);
                        }

                        shinst.common.src2.InitializeFromTypeAndIndex(arguments[2].GetType(), arguments[2].GetIndex());
                        input_mask_src2 = arguments[2].mask;
                    }
                    input_mask_src1 = arguments[1].mask;

                    shinst.common.dest.InitializeFromTypeAndIndex(arguments[0].GetType(), arguments[0].GetIndex());
                    shinst.common.src1.InitializeFromTypeAndIndex(arguments[1].GetType(), arguments[1].GetIndex());

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
        } else if (statement.which() == 2) {
            auto& var = boost::get<StatementRegisterName>(statement);

//            if (var.size() < /*3*/2)
//                throw "Not enough arguments given for register name binding";

//            if (var[/*1*/0].which() != Token::Identifier || var[/*2*/1].which() != Token::Register)
//                throw "Invalid arguments given for register name binding (got " + std::to_string(var[/*1*/0].which()) + " and " + std::to_string(var[/*2*/1].which()) + ", expected name and register, e.g. \".out_pos position o1\")";

//            if (boost::fusion::at_c<1>(var[/*2*/1].GetRegister()) != boost::none)
//                throw "Specifying a swizzler mask for binding register names is forbidden";

/*            if (var.GetType() == StatementRegisterName::Constant) {
                if (var.size() < 4)
                    throw "Not enough arguments given for constant assignment";

                if (var[3].which() != Token::Constant)
                    throw "Invalid arguments given for register name binding (expected name, register and value, e.g. \".const my_vector (0.4,0.2,0.1,0.0)\")";
            }*/

//            context.register_symbols.add( var.GetName().c_str(), var.GetRegisterWithIndex() );
            Atomic ret = identifiers[boost::fusion::at_c<2>(var)];
            Identifier new_identifier = identifiers.size();
            identifiers.push_back(ret);
            context.identifiers.add(boost::fusion::at_c<1>(var), new_identifier);

            // TODO: Write names to output shbin
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
        std::cout << "At: " << input_code.substr(preparse_begin - input_code.begin(), begin - preparse_begin) << std::endl;
        return 1;
    } catch(const char* err) {
        std::cout << "Error: " << err << std::endl;
        std::cout << "At: " << input_code.substr(preparse_begin - input_code.begin(), begin - preparse_begin) << std::endl;
        return 1;
    }

    return 0;
}
