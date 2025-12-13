# z8086: FPGA 8086 Core Powered by Original Microcode

**z8086** is a clean-room, FPGA-targeted implementation of the Intel 8086/8088 microprocessor. It executes the original 8086 microcode and provides a simplified memory/IO interface for FPGA integration.

## Overview

The z8086 core is a faithful implementation of the 8086 microprocessor architecture, using the original Intel microcode to ensure instruction-level compatibility. The core is written in SystemVerilog and designed to be vendor-agnostic (no FPGA-specific primitives).

### Key Features

- **Microcode-driven**: Executes the original 8086 microcode (21-bit words) loaded from `ucode.hex`
- **Simplified bus interface**: `rd/wr/io/word` with `ready` handshake; unaligned word accesses are automatically split internally
- **Prefetch queue**: 3-word (6-byte) prefetch queue with suspend/flush/correct-IP support
- **Bit-slice ALU**: Explicit 16×1-bit slice design with accurate flags (DAA/DAS/AAA/AAS, shifts/rotates, etc.)
- **FPGA-friendly**: Single clock, synchronous design, no vendor primitives

This is the first experimental release. The core passes all single-instruction tests, achieving a 100% pass rate (16,150 out of 16,150 vectors) with full 8086 instruction set coverage—including arithmetic, logic, shifts, control flow, string operations, multiplication/division, interrupts, I/O, and BCD/ASCII adjust instructions. FPGA synthesis has been validated on both Gowin GW5A and Xilinx Artix-7 platforms. Note that current limitations include non-cycle-accurate timing, no LOCK pin support, and no coprocessor interface.

**Resource usage**: ~1800 LUTs on Xilinx Artix7, ~1179 ALMs on Altera Cyclone V, ~2500 LUTs on Gowin GW5A. One BRAM block is used for the microcode. The code is about 1800 lines of SystemVerilog.

### Architecture

![](doc/z8086.svg)

The z8086 maintains the logical distinction between BIU (Bus Interface Unit) and EU (Execution Unit) but implements them in a unified datapath. The core includes:

- **Microcode Engine**: Fetches micro-instructions from the 512-word ROM (with address compression) and orchestrates all key CPU operations
- **Loader State Machine**: Manages instruction fetch timing (FC/SC signals)
- **Prefetch Queue**: 6-byte circular buffer for instruction prefetching
- **ALU**: 16-bit arithmetic/logic unit with comprehensive flag support
- **Memory Interface**: Unified bus FSM that arbitrates EU data operations vs instruction fetch

<!-- For detailed architecture documentation, see [`doc/z8086.md`](../../doc/z8086.md). -->

## Source Files

The core implementation is located in `src/`:

- **`src/z8086/z8086.sv`**: Main CPU core
- **`src/z8086/alu.sv`**: 16-bit ALU and flag computation
- **`src/z8086/ucode.hex`**: Microcode ROM image (512 × 21-bit words)
- **`src/soc/z8086_top.sv`**: Example SoC top-level with memory interface
- **`src/soc/spram.sv`**: Simple single-port RAM for memory subsystem

## Simulation Testing

To run simulations, build the Verilator testbench and use the provided Python runners in `tests/`. Prerequisites: Verilator 4.0+, C++ compiler, make, Python 3.

**Build simulator:**
```bash
cd 04.z8086/tests
make -s build
```

**Test an instruction:**
```bash
python test8088.py -f 8088/10.json -v    # single opcode/file
```

- `-f <file>`: test file (JSON)
- `-v`: verbose
- `-i <case>`: specific test case

**Test all instructions:**
```bash
python test8088.py
```

**Run small program tests:**
```bash
python test186.py
```

**Run interrupt test:**
```bash
make sim-int
```
(Exercises the INTA cycle and vector fetch.)

## FPGA Board Testing

FPGA board projects are in the `boards/` directory:

- **Tang Console 60K (Gowin GW5AT-60B)**: `z8086_console60k.gprj` (Gowin IDE). Drives LED pmod or Muse Labs LCD pmod. Press S1 button to start.
- **MicroPhase XC7A35T (Artix-7)**: `z8086.xpr` (Vivado 2021.1). Drives onboard LED. For LCD, see photo linked below for connections. Press K1 button to start.
- **DE10-Nano (Cyclone V)**: `z8086.qpf` (Quartus Prime). Drives onboard LED. For LCD, see photo linked below for connections. Press KEY0 to start.

See LCD demo screenshots: DE10-Nano, [Tang Console 60K](doc/tang_lcd.jpg), [MicroPhase Artix7](doc/microphase_lcd.jpg), [DE10-Nano](doc/de10nano_lcd.jpg).

## Example SoC Integration

The `src/soc/z8086_top.sv` module shows how to integrate the z8086 core:

```systemverilog
z8086 cpu (
    .clk(clk50),
    .reset_n(cpu_reset),
    .addr(addr),
    .din(din),
    .dout(dout),
    .wr(wr),
    .rd(rd),
    .io(io),
    .word(word),
    .ready(ready),
    .intr(1'b0),
    .nmi(1'b0),
    .inta()
);

// Memory subsystem
spram #(.AWIDTH(16)) ram (
    .clk(clk50),
    .addr(addr[15:0]),
    .din(dout),
    .dout(din),
    .we(wr & ~io),
    .oe(rd & ~io)
);
```

Here are the memory interface:

**Signals:**
- `addr[19:0]`: 20-bit physical address
- `din[15:0]`: Data input from memory/IO
- `dout[15:0]`: Data output to memory/IO
- `rd`: Read strobe (active high)
- `wr`: Write strobe (active high)
- `io`: 1=I/O access, 0=memory access
- `word`: 1=16-bit access, 0=8-bit access
- `ready`: Memory/IO ready (handshake)

**Bus Protocol:**
1. The z8086 core asserts `rd` (read) or `wr` (write) along with valid `addr`, `word`, and `io` signals.
2. The external memory or I/O subsystem initiates and completes the requested operation.
3. When the operation is finished, the external logic sets `ready=1`. For reads, data must be present on `din`.
4. The core automatically splits unaligned word accesses into two sequential byte operations. External systems will **never see** unaligned word accesses (`word` and `addr[0]` will not both be 1).
5. **Interrupt Handling:** When INTR is raised, the core triggers two INTA (interrupt acknowledge) bus cycles in accordance with the 8086 protocol. For each INTA cycle, the CPU asserts INTA. After the second INTA, external logic should place the interrupt vector number on `din` and assert `ready=1` to complete the transaction.

## Programming the z8086

The example SoC in `z8086_top.sv` and the `programs/` directory demonstrate one way of programming the core, backed by FPGA BRAM.

### Firmware Build & Load Flow

1. **Write your firmware:** Create your program in 8086 assembly language. You can find example programs in the `programs/` directory.
2. **Build the firmware:**
   - Use the Makefile in `programs/` to automate the build process. The Makefile invokes [NASM](https://www.nasm.us/) to assemble `.asm` sources into `.bin` binaries.
   - These binaries are then converted to `.hex` format for FPGA loading.
3. **Understanding the memory map and program loading:**
   - The system is configured with 128KB of main memory. Even-numbered segments map to the lower 64KB ("data segment"), while odd-numbered segments map to the upper 64KB ("program segment").
   - When building, the `program.hex` output places an empty 64KB in the first half, and the assembled program in the second 64KB. 
   - The SoC module `z8086_top.sv` uses `$readmemh` to load the `.hex` directly into FPGA RAM.
   - A reset vector trampoline (far jump to F000:0000) is at the end of the second page (offset FFF0).
4. **Boot procedure:** Upon reset, the CPU sets CS:IP to FFFF:0000 and executes that far jump. Execution then continues from F000:0000, the start of the user program, which is correctly mapped to the program segment.

### Example Programs

Several example firmware files are included in the `programs/` directory:
- `blinky.asm`: Minimal example toggling LEDs and counting up.
- `lcd_bars.asm`: Demonstrates basic LCD display functionality.
- `lcd_shapes.asm`: Repeated square drawing.

Refer to these examples as starting points for your own programs, and consult `z8086_top.sv` for detailed integration.

## Documentation

Coming soon.

<!-- - **[`doc/z8086.md`](../../doc/z8086.md)**: Comprehensive developer guide covering architecture, microcode, implementation details, and signal reference -->

## Acknowledgements

z8086 would not have been possible without the outstanding reverse engineering work of [Ken Shirriff](https://www.righto.com/search/label/808) and [Andrew Jenner](https://www.reenigne.org/blog/8086-microcode-disassembled/).

## Cite

```bibtex
@misc{z8086,
  title        = {z8086: FPGA 8086 Core Powered by Original Microcode},
  author       = {nand2mario},
  year         = {2025},
  url          = {https://github.com/nand2mario/z8086}
}
```

## Can I use this in a commercial project?

z8086 was developed primarily as an educational and learning tool. For broader or commercial use, the main limitation is the copyright status of the original 8086 microcode. Ideally, Intel would formally release the rights to this significant historical code.

As for z8086 itself, I have chosen to release it under the permissive Apache 2.0 license.

