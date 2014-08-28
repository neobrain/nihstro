# nihstro - 3DS vertex shader tools

nihstro is a collection of tools for 3DS vertex shaders targeted at homebrew development and/or reverse engineering. Currently, it supports assembling 3DS shader binaries from assembly source code and disassembling shaders from `shbin` files.

This project is released under a three-clause BSD license, for details see the license.txt.

## Status
The application itself works stable and very reliably. Basic arithmetic in shaders can be disassembled just fine, however any more advanced instructions (e.g. flow control) have not been implemented so far.

## Building
A compiler supporting C++11 is required. You will also need CMake to generate build files, however it is simple enough to compile the source code manually. If you want to use the assembler, you will also need to have `boost::spirit` installed.

## Command line usage
`nihstro-assemble <output_filename.shbin> <input_filename.vsh>`

Reads vertex shader source code from the input file and generates a shader binary from it.

`nihstro-disassemble <filename.shbin>`

Parses the shader binary header and outputs basic information on the DVLE sections.

`nihstro-disassemble <filename.shbin> <DVLE index>`

Parses the shader binary header and outputs basic information, but also disassembles the shader code using the information in the indexed DVLE (main offset, symbols, etc).

## Contributing
I welcome any contributions! Just create a GitHub fork and submit your changes back via pull requests.

## Kudos
A big "thank you!" to everyone who contributed to the information on 3dbrew, which has proven amazingly useful for my 3DS related projects. Another shout-out goes to smealum's aemstro, which served as a great reference when debugging nihstro-disassemble.
