# nihstro - a 3DS shader disassembler and shader binary parser

nihstro is a parser and disassembler for 3DS shader binary (shbin) files, written in C++.

This project is released under a three-clause BSD license, for details see the license.txt.

## Status
The application itself works stable and very reliably. Basic arithmetic in shaders can be disassembled just fine, however any more advanced instructions (e.g. flow control) have not been implemented so far.

## Building
A compiler supporting C++11 is required. Other than that a CMake build system is provided for convenience, but alternatively you can just compile the file `main.cpp` manually.

## Command line usage
Usage: `nihstro <filename.shbin>`
This will parse the shader binary header and output basic information on the DVLE sections.

Usage: `nihstro <filename.shbin> <DVLE index>`
This also parses the shader binary and outputs basic information, but also disassembles the shader code using the information in the indexed DVLE (main offset, symbols, etc).

## Contributing
I welcome any contributions! Just create a GitHub fork and submit your changes back via pull requests.

## Kudos
A big "thank you!" to everyone who contributed to the information on 3dbrew, which has proven amazingly useful for my 3DS related projects. Another shout-out goes to smealum's aemstro, which served as a great reference when debugging nihstro.
