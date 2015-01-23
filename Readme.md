# nihstro - 3DS shader tools

nihstro is a collection of tools for 3DS shaders targeted at homebrew development and/or reverse engineering. Currently, it supports assembling 3DS shader binaries from assembly source code and disassembling shaders from `shbin` files and provides C++ interfaces for analyzing and runtime-compiling shaders.

This project is released under a three-clause BSD license. For details see license.txt.

## Components

nihstro is well-modularized to minimize dependencies for the intended use case.

### Shader assembler
nihstro includes a standalone shader assembler for generating SHBIN files from human-readable shader source code (the syntax of which is called "nihcode"). This assembler is still in its infancy, but should work fine for simple shaders performing basic arithmetic. Shader source needs to follow the [nihcode specification](docs/nihcode_spec.md).

Usage:
`nihstro-assemble <output_filename.shbin> <input_filename.vsh>`

Reads vertex shader source code from the input file and generates a shader binary from it.

### Shader disassembler

nihstro includes a standalone shader disassembler is stable and works reliably. Basic arithmetic in shaders can be disassembled just fine, however any more advanced instructions (e.g. flow control) have not been implemented so far, and hence will be printed as unknown instructions.

Usage:
`nihstro-disassemble <filename.shbin>`

Parses the shader binary header and outputs basic information on the DVLE sections.

`nihstro-disassemble <filename.shbin> <DVLE index>`

Parses the shader binary header and outputs basic information, but also disassembles the shader code using the information in the indexed DVLE (main offset, symbols, etc).

### Shader bytecode and SHBIN C++ headers
The header `shader_bytecode.h` defines C++ data structures used in raw shader binary data, while `shader_binary.h` defines the layout of SHBIN files. This allows for convenient inspection of data in C++ code.

### Inline assembler
The header `inline_assembly.h` provides means for runtime generation of PICA200 shaders and SHBIN files within C++ code, so that homebrew authors don't need to ship shaders as precompiled files. While you could use nihstro's actual assembler to allow for runtime shader compilation, the inline assembler is more efficient and lighter on dependencies (since it doesn't require Boost to function).

A simple [example program](examples/inline_assembler/simple) is included to illustrate how to use the inline assembler.

## Building
All nihstro components require compiler support for C++11 to work.

For the standalone assembler and disassembler, you will also need CMake to generate build files, however it is simple to compile the source code manually if need be. If you want to use the standalone assembler, you will need to have `boost::spirit` installed (as well as some other more common Boost libraries).

The data structures defined by nihstro and the inline assembler are header-only and hence easy to integrate into any build system as long as nihstro's directory structure is preserved.

## Contributing
I welcome any contributions! Just create a GitHub fork and submit your changes back via pull requests.

## Kudos
A big "thank you!" to everyone who contributed to the information on 3dbrew, which has proven amazingly useful for my 3DS related projects. Another shout-out goes to smealum's aemstro, which served as a great reference when debugging nihstro-disassemble.
