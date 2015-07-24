// Copyright 2015 Tony Wasserka
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

#include <nihstro/parser_assembly_private.h>
#include <nihstro/preprocessor.h>
#include <nihstro/source_tree.h>

#include <boost/spirit/include/qi.hpp>

#include <fstream>

namespace nihstro {

template<typename Iterator>
struct IncludeParser : qi::grammar<Iterator, std::string(), AssemblySkipper<Iterator>> {
    using Skipper = AssemblySkipper<Iterator>;

    IncludeParser() : IncludeParser::base_type(include) {
        include = qi::lexeme[qi::lit(".include") >> &qi::ascii::space]
                  > qi::lexeme[qi::lit("\"") > +qi::char_("a-zA-Z0-9./_\\-") > qi::lit("\"")]
                  > qi::omit[qi::eol | qi::eoi];
    }

    qi::rule<Iterator, std::string(), Skipper> include;
};


SourceTree PreprocessAssemblyFile(const std::string& filename) {
    SourceTree tree;
    tree.file_info.filename = filename;

    std::ifstream input_file(filename);
    if (!input_file) {
        throw std::runtime_error("Could not open input file " + filename);
    }

    std::string prefix;
    {
        auto last_slash = filename.find_last_of("/");
        if (last_slash != std::string::npos)
            prefix = filename.substr(0, last_slash + 1);
    }

    input_file.seekg(0, std::ios::end);
    tree.code.resize(input_file.tellg());

    input_file.seekg(0, std::ios::beg);
    input_file.read(&tree.code[0], tree.code.size());
    input_file.close();

    auto cursor = tree.code.begin();

    IncludeParser<decltype(cursor)> include_parser;
    AssemblySkipper<decltype(cursor)> skipper;

    while (cursor != tree.code.end()) {
        std::string parsed_filename;
        auto cursor_prev = cursor;
        if (qi::phrase_parse(cursor, tree.code.end(), include_parser, skipper, parsed_filename)) {
            if (parsed_filename[0] == '/')
                throw std::runtime_error("Given filename must be relative to the path of the including file");

            // TODO: Protect against circular inclusions
            auto newtree = PreprocessAssemblyFile(prefix + parsed_filename);
            tree.Attach(newtree, cursor_prev - tree.code.begin());
            cursor = tree.code.erase(cursor_prev, cursor);
            cursor = tree.code.insert(cursor, '\n');
        } else {
            // Skip this line
            qi::parse(cursor, tree.code.end(), *(qi::char_ - (qi::eol | qi::eoi)) >> (qi::eol | qi::eoi));
        }
    }
    return tree;
}

} // namespace
