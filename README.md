# verilog-65C2402-fsm
A verilog model of the mythical 65C2402 CPU, a 24-bit address version of the WDC 65C02

## Design goals
The main design goal is to show the possibility of a backwards-compatible 65C02 with a 24-bit
address bus, with no modes, no new flags, just two new op-codes: CPU and A24

$0F: CPU isn't necessary, but fills the A register with #$10, matching the prefix code
$1F: A24 does nothing by itself.  Like in the Z80, it's a prefix code that modifies the
subsequent opcode.

When prefixed all ABS / ABS,X / ABS,Y / IND, and IND,X opcodes take a three byte address in the
subsequent three bytes.  E.g. $1F $AD $EF $78 $56 = LDA $5678EF.

Opcode A24 before a JMP or JSR changes those opcodes to use three bytes to specify the address,
with this 24-bit version of JSR pushing three bytes onto the stack: low, high, 3rd.  The matching
24-bit RTS ($1F $60) pops three bytes off the stack low, high, and 3rd.

RTI always pops four bytes: low, high, and 3rd for the IR, then 1 byte for the flags
(But Arlet's code doesn't support IRQ or NMI, so this CPU never pushes those bytes)

The IRQ, RST, and NMI vectors are $FFFFF7/8/9, $FFFFFA/B/C, and $FFFFFD/E/F.

Without the prefix code, all opcodes are identical to the 65C02.  Zero page is unchanged.
ABS and IND addressing are all two bytes.  Historic code using JSR/RTS will use 2-byte/16-bit
addresses.

The only non-backward-compatible behaviors are the new interrupt vectors. A new RST handler
could simply JMP ($FFFC), presuming a copy of the historic ROM was addressable at in page $FF.
A new IRQ handler similarly JMP ($FFFE).  The only issue would be legacy interrupt handlers
that assumed the return address was the top two bytes on the stack, rather than three.

## Changes from the original

PC (the program counter) is extended from 16-bits to 24-bits 
AB (the address bus) is extended from 16-bits to 24-bits
D3 (a new data register) is added to allow loading three-byte addresses

One new decode line is added for pushing the third byte for the long JSR

A handful of new states were added to the finite state machine that process the opcodes, in general
just one new state for handling ABS addresses, three-byte JMP/JSR, and three-byte RTS/RTI

## The hypothetical roadmap

A potential next step design is to further extend the address bus to 32-bits and 48-bits. 
In keeping with this core design, new prefix codes $2F and $3F would be used.

Along similar lines, the data bus and A, X, Y, and S registers could be extended to 16-bits,
24-bits, and 32-bits, using prefix codes $4F, $8F, and $BF to specify the width for each
instruction.

The CPU (0F) opcode would fill the A register with #$10, #$20, or #$30 depending on the
widest address bus supported, logically or'd with #$40, #$80, or #$B0 to specify the widest
data bus supported.  E.g., #50 = 16-bit data bus/registers and 24-bit address bus.

## Building with and without the testbed

main.v, ram.v, ram.hex, and vec.hex are the testbed, using the SIM macro to enable simulations.
E.g. iverilog -D SIM -o test *.v; vvp test

ram.hex is 128K, loaded from $000000-$01ffff.  Accessing RAM above $020000 returns x's.
vec.hex are the NMI, RST, and IRQ vectors, loaded at $FFFFF0-$FFFFFF (each is three bytes)

Use macro ONEXIT to dump the contents of RAM 16-bytes prior to the RST vector and 16-bites starting
at the RST vector before and after running the simulation.  16-bytes so that you can use those
bytes as storage in your test to check the results.

The opcode HLT (#$db) will end the simulation.


# Based on Arlet Ottens's verilog-65C02-fsm
## (Arlet's notes follow)
A verilog model of the 65C02 CPU. The code is rewritten from scratch.

* Assumes synchronous memory
* Uses finite state machine rather than microcode for control
* Designed for simplicity, size and speed
* Reduced cycle count eliminates all unnecessary cycles

## Design goals
The main design goal is to provide an easy understand implementation that has good performance

## Code
Code is far from complete.  Right now it's in a 'proof of concept' stage where the address
generation and ALU are done in a quick and dirty fashion to test some new ideas. Once I'm happy
with the overall design, I can do some optimizations. 

* cpu.v module is the top level. 

Code has been tested with Verilator. 

## Status

* All CMOS/NMOS 6502 instructions added (except for NOPs as undefined, Rockwell/WDC extensions)
* Model passes Klaus Dormann's test suite for 6502 (with BCD *disabled*)
* BCD not yet supported
* SYNC, RST supported
* IRQ, RDY, NMI not yet supported

### Cycle counts
For purpose of minimizing design and performance improvement, I did not keep the original cycle
count. All of the so-called dead cycles have been removed.
(65C2402 has more cycles for prefixed opcodes, and counts below *include* A24 prefix)

| Instruction type | Cycles | 24-bit |
| :--------------: | :----: | :----: |
| Implied PHx/PLx  |   2    |        |
| RTS              |   4    |   6    |
| RTI              |   5    |   7    |
| BRK              |   7    |        |
| Other implied    |   1    |        |
| JMP Absolute     |   3    |   5    |
| JMP (Indirect)   |   5    |   8    |
| JSR Absolute     |   5    |   7    |
| branch           |   2    |        |
| Immediate        |   2    |        |
| Zero page        |   3    |        |
| Zero page, X     |   3    |        |
| Zero page, Y     |   3    |        |
| Absolute         |   4    |   6    |
| Absolute, X      |   4    |   6    |
| Absolute, Y      |   4    |   6    |
| (Zero page)      |   5    |        |
| (Zero page), Y   |   5    |        |
| (Zero page, X)   |   5    |        |

Add 1 cycle for any read-modify-write. There is no extra cycle for taken branches, page overflows, or for X/Y offset calculations.

Have fun. 
