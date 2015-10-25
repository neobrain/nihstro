# nihstro - 3DS shader tools

[![Travis CI Build Status](https://travis-ci.org/neobrain/nihstro.svg)](https://travis-ci.org/neobrain/nihstro)

nihstro is a collection of tools for 3DS shaders targeted at homebrew development and/or reverse engineering. Currently, it supports assembling 3DS shader binaries from assembly source code and disassembling shaders from `shbin` files. It also provides C++ interfaces for analyzing and runtime-compiling shaders.

This project is released under a three-clause BSD license. For details see license.txt.

## Components

nihstro is well-modularized to minimize dependencies for any particular use case.

### Shader assembler
nihstro includes a standalone shader assembler for generating [SHBIN](http://3dbrew.org/wiki/SHBIN) files from human-readable shader source code (the syntax of which is called "nihcode"). It is perfectly suitable for homebrew development. Shader source needs to follow the [nihcode specification](docs/nihcode_spec.md).

Usage:
`nihstro-assemble <input_filename.vsh> -o <output_filename.shbin>`

Reads vertex shader source code from the input file and generates a shader binary from it.

Further command line options:
* `-h, --help`: Show command line usage
* `-i, --input`: Explicit switch for specifying the input shader source filename (if omitted, the first switch-less argument is interpreted as the filename)
* `-o, --output`: Output shbin filename (required)
* `-e, --entrypoint`: label name in the input source at which shader execution should start (defaults to "main")
* `-g, --geo_shader`: Compile shader source as a geometry shader

### Shader disassembler

nihstro includes a standalone shader disassembler for disassembling SHBIN files and inspecting meta data (symbol information, constant values, etc).

Usage:
`nihstro-disassemble <filename.shbin>`

Parses the shader binary header and outputs basic information on the DVLE sections.

`nihstro-disassemble <filename.shbin> <DVLE index>`

Parses the shader binary header and outputs basic information, but also disassembles the shader code using the information in the indexed DVLE (main offset, symbols, etc).

### Shader bytecode and SHBIN C++ headers
The header `shader_bytecode.h` defines C++ data structures which describe raw shader binary code, while `shader_binary.h` defines the layout of SHBIN files. This allows for convenient inspection of data in C++ code. Note that these headers are currently not API stable.

### Inline assembler (experimental)
The header `inline_assembly.h` provides an experimental mean for runtime generation of PICA200 shaders and SHBIN files within C++ code, so that homebrew authors don't need to ship shaders as precompiled files. While you could use nihstro's actual assembler to allow for runtime shader compilation, the inline assembler may be more convenient and is lighter on dependencies (since it doesn't require Boost to function). However, for obvious reasons it incurs a performance penalty and an increased memory usage compared to offline shader compilation.

A simple [example program](examples/inline_assembler/simple) is included to illustrate how to use the inline assembler.

Note that the inline assembler is highly experimental. It may or may not work for you yet, and its API will change a lot in the future.

## Building

All nihstro components require compiler support for C++11 to work.

The C++ headers `shader_bytecode.h` and `shader_binary` can be easily be included in other project and hence are easy to integrate into any build system (as long as nihstro's directory structure is preserved).

For the standalone assembler and disassembler, you will also need CMake to generate build files (however it is simple to setup a different build system from scratch if need be), and at least parts of the [Boost libraries](http://www.boost.org/) installed (including Spirit, Fusion, and others).

### Installing dependencies on OS X

On OS X, it is recommended that you use [Homebrew](http://brew.sh/) to install dependencies. You'll need to run the following to build nihstro:

```
brew install cmake boost
```

### Compiling

To compile the standalone assembler and disassembler, run:

```
mkdir -p build
cd build
cmake ..
make
```

This will build the `nihstro-assemble` and `nihstro-disassemble` standalone executables inside the `build` directory. 

## Contributing
I welcome any contributions! Just create a GitHub fork and submit your changes back via pull requests.

## Kudos
A big "thank you!" to everyone who contributed to the information on 3dbrew, which has proven amazingly useful for my 3DS related projects. Another shout-out goes to smealum's aemstro, which served as a great reference when debugging nihstro.
