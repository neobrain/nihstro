# Shader Instruction Set

This page gives an overview over the instruction set supported by nihstro. Note that there is a similar reference list on [3dbrew](http://3dbrew.org/wiki/Shader_Instruction_Set), which documents the actual implementation on hardware though. nihstro seeks to abstract away annoying details like the fact that there are 3 different CALL instructions, and instead provides convenience shortcuts where possible without giving up flexibility.

# Table of Contents

- [Shader Instruction Set](#shader-instruction-set)
  - [Arithmetic Instructions](#arithmetic-instructions)
  - [Flow Control Instructions](#flow-control-instructions)
  - [Special Purpose Instructions](#special-purpose-instructions)

## Arithmetic Instructions
Most arithmetic instructions take a destination operand and one or more source operands. Source operands may use any kind of swizzle mask, while destination operands may not use reordering or duplicating swizzle masks. Below you will find a short operation description for each instruction, e.g. `dest[i] = src[i]`, which means that the `i`-th source component (as specified by the swizzle mask) will be assigned to the `i`-th destination component (as specified by the swizzle mask), with `i` ranging from 1 to the number of swizzle mask components. Components not listed in the destination swizzle mask hence will not be written.

Static indexing (i.e. indexing with a constant, not to be confused with the above notation) may be done for both operand types. Source operands additionally support *dynamic indexing*, where the index depends on one of the address registers `a0`/`a1` or on the loop counter `lcnt`. Examples:
* static indexing: `c0[20]`
* dynamic indexing: `c0[2+a0]`

#### mov: Copy floating point value
Syntax: `mov dest_operand, src_operand`

Operation: `dest[i] = src[i]`

Restrictions:
* `src` and `dest` must have the same number of components

#### add: Per-component floating point sum
Syntax: `add dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = src1[i] + src2[i]`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

Notes:
* subtraction can be performed using negation: `add r0, c0, -c1`
* when chaining an addition and a multiplication, consider using `mad` instead

#### mul: Per-component floating point multiplication
Syntax: `mul dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = src1[i] * src2[i]`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

Notes:
* division can be performed by computing the reciprocal of src2 and multiplying the result: `rcp r0, c1; mul r0, c0, r0`
* when chaining an addition and a multiplication, consider using `mad` instead

#### mad: Fused multiply-add of three floating point numbers
Syntax: `mad dest_operand, src1_operand, src2_operand, src3_operand`

Operation: `dest[i] = src1[i] * src2[i] + src3[i]`

Restrictions:
* `src1`, `src2`, `src3`, and `dest` must have the same number of components
* not more than two source operands may be float uniform registers
* no dynamic indexing may be performed on any of the source operands.

Notes:
* when dynamic indexing is not avoidable, use `add` and `mul` instead
* not supported currently

#### max: Copy the greater of two floating point numbers
Syntax: `max dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = max(src1[i], src2[i])`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

#### min: Copy the smaller of two floating point numbers
Syntax: `min dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = min(src1[i], src2[i])`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

#### flr: Floating point floor
Syntax: `flr dest_operand, src_operand`

Operation: `dest[i] = floor(src[i])`

Restrictions:
* `src` and `dest` must have the same number of components

#### rcp: Floating point reciprocal
Syntax: `rcp dest_operand, src_operand`

Operation: `dest[i] = 1 / src[i]`

Restrictions:
* `src` and `dest` must have the same number of components

#### rsq: Floating point reciprocal square root
Syntax: `rsq dest_operand, src_operand`

Operation: `dest[i] = 1 / sqrt(src[i])`

Restrictions:
* `src` and `dest` must have the same number of components

#### exp: Floating point base-2 exponential
Syntax: `exp dest_operand, src_operand`

Operation: `dest[i] = exp(src[i])`

Restrictions:
* `src1` and `dest` must have the same number of components

#### log: Floating point base-2 logarithm
Syntax: `log dest_operand, src_operand`

Operation: `dest[i] = log(src[i])`

Restrictions:
* `src1` and `dest` must have the same number of components

#### dp3: Floating point 3-component dot-product
Syntax: `dp3 dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = src1[0]*src2[0]+src1[1]*src2[1]+src1[2]*src2[2])`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

#### dp4: Floating point 4-component dot-product
Syntax: `dp4 dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = src1[0]*src2[0]+src1[1]*src2[1]+src1[2]*src2[2]+src1[3]*src2[3])`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

#### dph: Floating point homogeneous dot-product
Syntax: `dph dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = src1[0]*src2[0]+src1[1]*src2[1]+src1[2]*src2[2]+src2[3]`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing.

#### sge: Set to one if greater or equal
Syntax: `sge dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = (src1[i] >= src2[i]) ? 1.0 : 0.0`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

#### slt: Set to one if (strictly) less
Syntax: `slt dest_operand, src1_operand, src2_operand`

Operation: `dest[i] = (src1[i] < src2[i]) ? 1.0 : 0.0`

Restrictions:
* `src1`, `src2`, and `dest` must have the same number of components
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

#### mova: Move to address register
Syntax: `mova src_operand`

Operation:

    a0 = src.x
    a1 = src.y

Restrictions:
* src_operand must be a two-component vector.

Notes:
* not supported currently

## Flow Control Instructions
These allow for non-linear code execution, e.g. by conditionally or repeatedly running code.

Some flow control instruction take a "condition" parameter. A condition is either
* a boolean uniform or
* an expression consisting of one or two conditional code components, combined via `&&` ("and") or `||` ("or"), and optionally negated. Examples: `cc.x`, `cc.y && !cc.x`

#### cmp: Compare two floating point numbers

Syntax: `cmp src1_operand, src2_operand, op1, op2`

`op1` and `op2` may be any of the strings `==` (equal), `!=` (not equal), `<` (less than), `<=` (less than or equal to), `>` (greater than), and `>=` (greater than or equal to).

Operation:

    cc.x = (src1[0] op1 src2[0])
    cc.y = (src1[1] op2 src2[1])

Restrictions:
* `src1` and `src2` must be two-component vectors
* it is not possible to set `cc.x` without also setting `cc.y`
* not more than one of the source operands may be a float uniform register and/or use dynamic indexing

Notes:
* this instruction is used to set conditional codes, which can be used as conditions for `if`/`jmp`/`call`/`break`.

#### if: Conditional code execution
Syntax: `if condition`

Operation:
If `condition` is true, conditionally executes the code between itself and the corresponding `else` or `endif` pseudo-instruction. Otherwise, executes the code in the `else` branch, if one is given (otherwise, skips the branch body and continues after the `endif` statement).

Restrictions:
* not more than one `else` branch may be specified (`else if` syntax is not supported)

Notes:
* all `if` branches must be closed explicitly using `endif`
* jumping out of a branch body may result in undefined behavior

Example:

    if cc.x && !cc.y
        // do stuff
    else
        if b0
            // do other stuff
        endif
    endif

#### loop: Repeat code execution
Syntax: `loop int_uniform`

Operation:
Initialize `lcnt` to `int_uniform.y`, then process code between `loop` and `endloop` for `int_uniform.x+1` iterations in total. After each iteration, `lcnt` is incremented by `int_uniform.z`.

Restrictions:
* no swizzle mask may be applied on the given uniform
* there is no direct way of looping zero times (the easiest way is to use `break` with an extra boolean uniform)

Notes:
* `lcnt` can be used to dynamically index arrays, e.g. to implement vertex lighting with multiple light sources

#### break: Break out of current loop
Syntax: `break condition`

Operation:
If `condition` is true, break out of the current loop.

Restrictions:
* jumping out of a branch body may result in undefined behavior

#### jmp: Jump to code address
Syntax: `jmp target_label if condition`

Restrictions:
* jumping out of or into branch bodies or loops may result in undefined behavior
* there is no way to force a jump without specifying a condition

Notes:
* if you need to automatically return from a function, use `call` instead

Example:

    main:
        jmp my_helper_code if b0
        // if not b0, do other stuff here
        nop
        end

    my_helper_code:
        // do stuff
        nop
        end

#### call: Jump to code address and return to caller
Possible syntaxes:
`call target_label until return_label if condition`
`call target_label until return_label`

Operation:
If `condition` is true (or none is given), jumps to `target_label` and processes shader code there until `return_label` is hit, at which point code execution jumps back to the caller.

Restrictions:
* jumping out of or into branch bodies or loops may result in undefined behavior

Notes:
* if you don't need to automatically return from a function, use `jmp` instead

Example:

    main:
        call my_helper_code until end_helper_code
        nop
        end

    my_helper_code:
        // do stuff here
        nop
    end_helper_code:

## Special Purpose Instructions
#### nop: No operation
Syntax: `nop`

Notes:
* This may be necessary before using `end` to make sure all pending write operations have been completed

#### end: Finish shader execution
Syntax: `end`

Operation:
Stops shader execution.
