#nihcode Specification

Version 0.1.

This page seeks to be a formal-ish specification of the input assembly language *nihcode* used by the nihstro shader assembler.

## Version information
This document is is intended to give developers an idea of how things are expected to work. Please file issue reports for any deviations in nihstro's behavior from this specifications that you find. Similarly, any inclarities in the specification will be corrected if reported, too.

## Structure
nihcode is a sequence of statements, each of which must be put onto a separate line. There are five types of statements:
* version information statements
* include statement
* alias declaration statements,
* label declaration statements, and
* instruction statements,
each of which is described in its own section below. Additionally, C++-like comments may be inserted at any point and are started using the character sequences `//`, `#`, or `;`. Comments span the rest of the line after any of these characters. Any statement must be written on its own line.

A pseudo-code example of nihcode looks like this:

    // First example shader
    .version 0.1                  // version information
    
    .alias inpos v0               // alias declaration
    .alias intex v1               // alias declaration
    .alias pos o0    as position  // alias declaration
    .alias pos o1.xy as texcoord0 // alias declaration

    .include "utils.h"            // include utility functionality

    main:                         // label declaration
        mov o0, v0                // instruction
        mov o1.xy, v1.xy          // instruction
        nop                       // instruction
        end                       // instruction


## Shader Registers, builtin Identifiers, Swizzle Masks
A shader can access a number of different registers with different purposes. *Input registers* expose the raw input vertex attribute data, while the output vertex attributes used for rendering is written to *output registers*. External programs can pass parameters to the shader by setting *uniforms*. Additionally, a number of *temporary registers* are free for any use. There are also special-purpose registers, namely the *address registers* and the *conditional code register*.

Registers are being referred to by using *identifiers*. There is a number of builtin identifiers, each of which refers to one register. Note that most registers are vectors, i.e. they comprise multiple components, which are accessed using swizzle masks.
* `v0`-`v15`: Input registers (read-only), four-component vectors
* `r0`-`r15`: Temporary registers (read-write), four-component vectors
* `c0`-`c95`: Float uniforms (read-only), four-component vectors
* `i0`-`i3`:  Integer uniforms (read-only), four-component vectors
* `b0`-`b15`: Boolean uniforms (read-only), scalar
* `o0`-`o15`: Output registers (write-only), four-component vectors
* `a0, a1, aL`: Address registers (used with MOVA and dynamic indexing), scalar
* `cc`: Conditional code register (used with CMP and flow-control instructions), two-component vector

For better readability, one can also define new identifiers, as explained below. Identifiers may only use a restricted set of names including lower- or uppercase letters a-Z, underscores, and decimal digits (the latter two which may not be used as the first character of the name). Additionally, an identifier may be followed by a swizzle mask, separated by the character `.` (e.g. `texcoord.zyx`). Swizzle masks allow for reordering, duplicating, and removing of one or more vector components of the identified register (without actually modifying that register).

When used with certain instructions, identifiers may be mentioned along with a sign, an array index, and/or a swizzle mask. Constructs like this are called *expressions*.

The following names are reserved identifiers, and may not be used during declarations:
* Any names starting with a `gl_` prefix
* Any names starting with a `dmp_` prefix
* Any names starting with an underscore prefix
* Any of the instruction opcodes mentioned below may not be used for the identifier name

## Aliases
### Plain Aliases (any register)
`.alias <new_identifier> <existing_identifier>{.<swizzle_mask>}`

Declares a new identifier called `new_identifier` which will refer to the same register that `existing_identifier` refers to, applying a swizzle_mask if specified. All subsequent uses of `new_identifier` are equivalent to using `existing_identifier.swizzle_mask`. Aliases of any register type may be created, however it should be noted that using output registers requires explicit assignment of an output semantic (see below).

E.g. `.alias input_texture v2.xy`

### Alias with Assignment of a Semantic (output registers)
`.alias <new_identifier> <existing_identifier>{.swizzle_mask} as <semantic>`

Declares an alias of `existing_identifier` with the name `new_identifier` and assigns the given semantic to the corresponding output register. An output semantic needs to be given to describe how the output vertex attribute is intended to be used after shader execution. `semantic` may be any of the strings `position`, `quaternion`, `color`, `texcoord0`, `texcoord1`, `texcoord2`, and `view`. If not all output register components are being written to, a swizzle mask should be used to denote the "active" components. Note that this swizzle mask may not reorder any components.

E.g. `.alias output_texcoord o1.xy as texcoord0`

### Constant Declarations (uniform registers)
scalar constants: `.alias <new_identifier> <existing_identifier> as <value>`

vector constants: `.alias <new_identifier> <existing_identifier> as (<x>, <y>{, <z>{, <w>}})`

Declares an alias of `existing_identifier` with the name `new_identifier` and assigns the given default value to it. Default values are parsed by the ctrulib API and automatically applied when enabling a shader. The number of components in the given constant must match the number of components in the specified register.

E.g. `.alias my_const c4 as (0.1, 3.2, -3.14, 0.0)`

## Label Declarations
`<labelname>:`

Declares a new label with the name `labelname` at the given source line, which can be used in flow control operations. Label names follow the same conventions as identifiers and may not share the same name with an existing identifier.

## Instruction Statements
Writes the given opcode according to the given arguments to the shader binary. There are a lot of instructions, and each of them uses one of the following formats:

Trivial operations:
`<opcode>`
Used by `else`, `emit`, `end`, `endif`, `endloop`, and `nop`.

Arithmetic operations:
`<opcode> <expression1>{, <expression2>{, <expression3>{, <expression4>}}}`
Used by `add`, `dp3`, `dp4`, `dph`, `ex2`, `flr`, `lg2`, `mad`, `max`, `min`, `mov`, `mova`, `mul`, `rcp`, `rsq`, `sge` and `slt`. The number of required expressions as well as their meaning depends on the opcode.

E.g. `mul o3.xyz c4.xyz v0.xyz`

Compare operation:
`cmp <expression1>, <expression2>, <op_x>, <op_y>`
Used exclusively by `cmp`. `expression1` and `expression2` must evaluate to two-component float vectors. `op_x` and `op_y` specify comparison operations for the x and y components of the given expressions, respectively. They may be `==`, `!=`, `<`, `<=`, `>` or `>=`.

E.g. `cmp c0.xy, i2.xy, <=, ==`

Flow control operations:
`<opcode> <condition>`
Used by `break`, `if` and `loop`.

`<opcode> {<target_label>} {until <return_label>} {if <condition>}`
Used by `jmp` and `call`.

 `condition` may either be an identifier of a boolean uniform or a conditional expression. Examples for conditional expressions are `cc.x`, `!cc.x`, `!cc.xy`, `cc.x && !cc.y`, and `cc.x || cc.y`, where `{!}cc.xy` is equivalent to `{!}cc.x && {!}cc.y`. `target_label` and `return_label` must be label identifiers. Their meaning depends on the given opcode.

For a full instruction set reference, go to [instruction set reference](instruction_set.md). You may also want to refer to [3dbrew](http://3dbrew.org/wiki/Shader_Instruction_Set) for low-level documentation on each opcode. Is is suggested that you take a look at the nihstro examples to get a better picture of how to apply that information.

## Include Statements
`.include "filename"`

Replaces the `.include` line with the contents of the given file. The filename is taken to be relative to the file it was included from.

## Version Information
`.version number`

This statement is a hint for the compiler to see which language specification the shader was written against. It may be used to toggle a compatibility assembling mode.

E.g. `.version 0.1`
