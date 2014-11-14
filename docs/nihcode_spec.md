#nihcode Specification

Version 0.0.1.

This page seeks to be a formal-ish specification of the input assembly language *nihcode* used by the nihstro shader assembler.

##Version information
This document is still heavily WIP and is intended to give developers an idea of how things are expected to work. Please file issue reports for any deviations in nihstro's behavior from this specifications that you find. Similarly, any inclarities in the specification will be corrected if reported, too.

## Structure
nihcode is a sequence of statements, each of which must be put onto a separate line. There are four types of statements:
* version information statements
* alias declaration statements,
* label declaration statements, and
* instruction statements,
each of which is defined in its own section below. Additionally, comments may be inserted at any point using C++ like comments, which are opened with `//` and span the rest of the line.

A pseudo-code example of nihcode looks like this:

    // First example shaders
    .alias input i0         // alias declaration
    .out pos o0 position // alias declaration

    main:                   // label declaration
        mov o0, i0          // instruction
        flush               // instruction
        ret                 // instruction


## Shader Registers, builtin Identifiers, Swizzle Masks
A shader can access a number of different registers with different meaning. The raw input vertex attribute data is read via *input registers*, while the output vertex attributes used for rendering is written to *output registers*. External programs can pass parameters to the shader by setting *uniforms*. Additionally, a number of *temporary registers* are free for any use. Each of these registers refers to a 4-component vector.

Registers are being referred to by using *identifiers*. There is a number of builtin identifiers, each of which refers to one register:
* `i0`-`i15`: Input registers (read-only)
* `t0`-`t15`: Temporary registers (read-write)
* `f0`-`f95`: Float uniforms (read-only)
* `o0`-`o15`: Output registers (write-only)

Purely for readability, one can also define new identifiers, as explained below. Identifiers may only use a restricted set of names including lower- or uppercase letters a-Z, underscores, and decimal digits (the latter two which may not be used as the first character of the name). Additionally, an identifier may be followed by a swizzle mask, separated by the character `.` (e.g. `texcoord.xzw`). Swizzle masks allow for reordering, duplicating, and removing of one or more vector components of the identified register (without actually modifying this register).

The following names are reserved identifiers, and may not be used during declarations:
* Any names starting with a `gl_` prefix
* Any names starting with a `dmp_` prefix
* Any names starting with an underscore prefix
* Any of the instruction opcodes mentioned below may not be used for the identifier name

## Variable Declarations
### Aliases (inputs, temporaries, uniforms)
`.alias new_identifier existing_identifier`

Declares a new identifier called `new_identifier` which refers to the same register that `existing_identifier` refers to. In particular, aliases can be used to give a compact name for a register and a fixed swizzle mask. Aliases may be used on any identifier, however it should be noted that identifiers for output registers should generally be defined via `.out` (see below).

E.g. `.alias input_texture i2.xy`

### Output Declarations (outputs)
`.out new_identifier existing_identifier semantic`

Declares a new identifier called `new_identifier` which refers to an output register given by `existing_identifier`. An output semantic needs to be given to describe how the output vertex attribute is intended to be used after shader execution. `semantic` can be any of the strings `position`, `color`, `texcoord0`, `texcoord1` and `texcoord2`.

E.g. `.out vtxpos o0.xyz position`

###constant declarations (uniforms)
scalar constants: `.const new_identifier existing_identifier value`

vector constants: `.const new_identifier existing_identifier (x, y(, z(, w)))`

Declares a new identifier called `new_identifier` which refers to a uniform register given by `existing_identifier`. Unless all components of the target register are given, `existing_identifier` must use a swizzle mask to specify how to store the constants. 

E.g. `.const my_const f4.xyw (0.1, 3.2, -3.14)`

##Label Declarations
`labelname:`

Declares a new label with the name `labelname` at the given source line, which can be used in flow control operations. Label names follow the same conventions as identifiers and may not share the same name with an existing identifier.

##Instruction Statements
`opcode expression(, expression(,expression(,expression)))`

Writes the given opcode to the shader binary. The number of required expressions as well as their meaning depends on the opcode.

An expression is constituted of an optional sign and an identifier.

E.g. `.mul o3.xyz f4.xyz i0.xyz`

##version information
`.version number`

This statement is a hint for the compiler to see which language specification the shader was written against. It may be used to toggle some compatibility assembling mode.

E.g. `.version 0.0.1`
