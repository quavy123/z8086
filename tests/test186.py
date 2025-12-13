#!/usr/bin/env python3
"""
Multi-Program Test Runner for z8086

Runs "80186_tests" programs and validates results against expected memory images.

Quirks of the 80186 test set:
* Most tests use a short jump (EB 0E) to jump to the start address, assuming CS=F000h and IP=FFF0h 
  (80286 behavior). Actual 8086 boots from FFFF:0000. We use plusargs to work around this.
* Many tests include flag values in the result. 8086 flags have many undefined bits (like bits 12–15). 
  The test runner ignores these bits in the comparison.
* Deleted `rotate.bin`, as it assumes CL is masked to 5 bits. 8086 uses the full 8 bits.
* `div.bin` was patched to remove instruction 0x1016. This was used to move the return IP after 
  the DIV instruction, which is not necessary on the 8086 as the return IP is the next instruction.
* `segpr.bin` was patched to skip div-by-0 return address fixing, and ignore specific flag bytes.

"""

import sys
import subprocess
import re
import argparse
from pathlib import Path


# Test programs directory
TESTS_DIR = Path("80186_tests")

# Verilator executable
VERILATOR_EXE = Path("obj_dir/Vtb_z8086")

# Test programs to run (in priority order)
TEST_PROGRAMS = [
    # Simple programs (basic instructions)
    "control",
    "datatrnf",
    "jmpmov",

    # Arithmetic
    "add",
    "sub",
    "cmpneg",

    # More arithmetic
    "mul",
    "div",
    "bcdcnv",

    # Bitwise and shifts
    "bitwise",
    # "rotate",
    "shifts",

    # Control flow
    "jump1",
    "jump2",
    "segpr",

    # Complex (may not pass yet)
    "strings",
    "rep",
    "interrupt",
]

# Per-program flag comparison configuration.
# Map: program -> { address(int): 'ODITSZAPC' }
FLAGS_CONFIG = {}
BYTES_IGNORE = {}

def _range_flags(start: int, end: int, step: int, letters: str):
    return {addr: letters for addr in range(start, end, step)}

def init_flags_config():
    """Initialize FLAGS_CONFIG with address->flags mappings by test name."""
    global FLAGS_CONFIG
    global BYTES_IGNORE
    FLAGS_CONFIG = {
        # bcdcnv: flags at 0x54,0x56,...,0x8e; only AF and CF are significant
        'bcdcnv': _range_flags(0x54, 0x90, 2, 'AC'),
        # cmpneg: flags at 0x5e..0x34 (even); CF OF SF ZF AF PF are significant
        'cmpneg': _range_flags(0x34, 0x60, 2, 'OCSZAP'),
        # datatrnf: FLAGS word at 0x0E-0x0F; check all flags. High nibble of 0x0F is ignored by mask
        'datatrnf': {0x0E: 'ODITSZAPC'},
        # control: flags at 0x00 and 0x02; all flags significant
        'control': {0x00: 'ODITSZAPC', 0x02: 'ODITSZAPC'},
        # mul: OF/CF across 0x88..0xbe (MUL/IMUL), at 0x88,0x8a,0x8c,0x8e test SF/ZF/PF instead (AAD)
        'mul': {
            **_range_flags(0x88, 0xC0, 2, 'OC'),
            **{a: 'SZP' for a in (0x88, 0x8A, 0x8C, 0x8E)},
        },
        # bitwise: flags at 0x98..0xfe (even addresses); all flags significant
        'bitwise': _range_flags(0x98, 0x100, 2, 'ODITSZAPC'),
        # shifts: flags at 0x44..0x7e (even); all except AF significant by default
        # Specific addresses ignore OF too
        'shifts': {
            **_range_flags(0x44, 0x80, 2, 'ODITSZPC'),
            **{a: 'DITSZPC' for a in (
                0x7A,0x78,0x76,0x74,0x6E,0x6C,0x66,0x64,
                0x62,0x60,0x5A,0x58,0x52,0x50,0x4E,0x4C,
                0x46,0x44
            )},
        },
        # div: flags at 0x94..0xce (even); all flags undefined (ignore all bits)
        'div': _range_flags(0x94, 0xD0, 2, ''),
    }
    # Per-test single-byte ignores (not full FLAGS words)
    BYTES_IGNORE = {
        # segpr: ignore specific bytes that carry undefined flag artifacts
        'segpr': [0x0E, 0x27, 0x2E],
    }
    return FLAGS_CONFIG

def _flags_to_word_mask(letters: str) -> int:
    """Map ODITSFZAPC letters to a 16-bit mask for FLAGS.

    Bits (8086 FLAGS): O D I T S Z - A - P - C (positions 11,10,9,8,7,6,4,2,0)
    Returns a 16-bit mask where selected flag bits are 1.
    """
    m = 0
    for ch in (letters or ''):
        ch = ch.upper()
        if ch == 'O':
            m |= (1 << 11)
        elif ch == 'D':
            m |= (1 << 10)
        elif ch == 'I':
            m |= (1 << 9)
        elif ch == 'T':
            m |= (1 << 8)
        elif ch == 'S':
            m |= (1 << 7)
        elif ch == 'Z':
            m |= (1 << 6)
        elif ch == 'A':
            m |= (1 << 4)
        elif ch == 'P':
            m |= (1 << 2)
        elif ch == 'C':
            m |= (1 << 0)
    # If no flags selected, mask is 0: ignore all bits at that location
    return m if m != 0 else 0x0000

def build_mask_map(program_name, length):
    cfg = FLAGS_CONFIG.get(program_name)
    mask_map = {}
    if not cfg:
        return mask_map
    for base, letters in cfg.items():
        word_mask = _flags_to_word_mask(letters)
        low_mask = word_mask & 0xFF
        high_mask = (word_mask >> 8) & 0xFF
        if 0 <= base < length:
            mask_map[base] = low_mask
        if 0 <= base + 1 < length:
            mask_map[base + 1] = high_mask
    return mask_map


def convert_bin_to_hex(bin_file, hex_file, verbose=False,
                       patch_addr=None, patch_bytes=None):
    """Convert .bin file to .hex using bin2hex.py"""
    cmd = ["python", "bin2hex.py", str(bin_file), "-o", str(hex_file)]
    if verbose:
        cmd.append("-v")
    # Forward optional patch args when provided
    if patch_addr is not None and patch_bytes is not None:
        cmd.extend(["--patch-addr", str(patch_addr), "--patch-bytes", patch_bytes])

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"bin2hex.py failed: {result.stderr}")

    return hex_file


def run_test_program(hex_file, max_cycles=1_000_000, result_addr=0, result_len=16, verbose=False):
    """
    Run test program in Verilator testbench.

    Args:
        hex_file: Path to .hex memory image
        max_cycles: Maximum simulation cycles
        result_addr: Address to read results from
        result_len: Number of bytes to read

    Returns:
        Tuple of (stdout, stderr, returncode)
    """
    cmd = [
        str(VERILATOR_EXE),
        f"+mem={hex_file}",
        f"+cycles={max_cycles}",
        f"+result_addr={result_addr}",
        f"+result_len={result_len}",
        f"+cs={0xF000}",
        f"+ip={0xFFF0}",
        f"+stop_after_first=0",  # multi-instruction programs should not stop after first instruction
        f"+flags_hi_f=0",        # 80186 tests expect F[15:12]=0 on POPF
    ]
    if verbose:
        print(f"  CMD: {' '.join(cmd)}")

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    return result.stdout, result.stderr, result.returncode


def extract_memory_from_output(output, addr=0):
    """
    Extract memory bytes from testbench output.

    Looks for lines like:
        RESULT MEM: @0=214
        RESULT MEM: @1=8
        RESULT MEM: @2=3
        RESULT MEM: @3=6

    Returns:
        List of byte values
    """
    pattern = rf"RESULT MEM: @([0-9a-fx]+)=(\d+)"
    matches = re.findall(pattern, output, re.IGNORECASE)

    # Build dict of address -> value
    mem = {}
    for addr_str, value_str in matches:
        addr_int = int(addr_str, 16) if 'x' in addr_str.lower() else int(addr_str)
        mem[addr_int] = int(value_str)

    # Convert to sequential list starting from requested address
    if not mem:
        return []

    max_addr = max(mem.keys())
    return [mem.get(i, 0) for i in range(addr, max_addr + 1)]


def validate_result(program_name, actual_bytes, expected_file):
    """
    Compare actual memory with expected result file.

    Returns:
        Tuple of (pass/fail bool, message string)
    """
    if not expected_file.exists():
        return False, f"Expected result file {expected_file} not found"

    expected = expected_file.read_bytes()

    if not actual_bytes:
        return False, "No memory output from testbench"

    # Compare up to length of expected result
    length = len(expected)

    if len(actual_bytes) < length:
        return False, f"Only got {len(actual_bytes)} bytes, expected {length}"

    # Build mask map for ignored bytes/nibbles
    mask_map = build_mask_map(program_name, length)
    # Apply per-test single-byte ignores (mask=0 only at the given byte)
    extra_ign = BYTES_IGNORE.get(program_name, [])
    for i in extra_ign:
        if 0 <= i < length:
            mask_map[i] = 0x00
    # Determine significant flags string for display by grouping addresses
    cfg = FLAGS_CONFIG.get(program_name, {})
    def _expand_flags(letters: str) -> str:
        order = ['O','D','I','T','S','Z','A','P','C']
        names = {'O':'OF','D':'DF','I':'IF','T':'TF','S':'SF','Z':'ZF','A':'AF','P':'PF','C':'CF'}
        have = set((letters or '').upper())
        s = ' '.join(names[ch] for ch in order if ch in have)
        return s if s else 'UNDEF'
    if cfg:
        groups = {}
        for addr, letters in cfg.items():
            groups.setdefault(letters, []).append(addr)
        parts = []
        for letters, addrs in sorted(groups.items(), key=lambda kv: kv[0]):
            addrs_sorted = ' '.join(f"0x{a:02x}" for a in sorted(addrs))
            parts.append(f"{addrs_sorted} -> {_expand_flags(letters)}")
        sig_str = '; '.join(parts) if parts else 'ALL'
    else:
        sig_str = 'ALL'
    # Find all mismatches under masking rules, and note ignored-but-different positions
    mismatches = []
    ignored_different = []  # positions where bytes differ but masked compare passes
    for i in range(length):
        mask = mask_map.get(i, 0xFF)
        act = actual_bytes[i]
        exp = expected[i]
        if (act & mask) != (exp & mask):
            mismatches.append(i)
        elif act != exp and i in mask_map:
            ignored_different.append(i)
    if mismatches:
        red = "\033[91m"; yellow = "\033[93m"; reset = "\033[0m"
        mis_set = set(mismatches)
        ign_set = set(ignored_different)
        rows = []
        header = f"Significant flags: {sig_str} (red=mismatch, yellow=ignored diff)"
        for base in range(0, length, 16):
            parts = []
            for i in range(base, min(base + 16, length)):
                eh = f'{expected[i]:02x}'
                ah = f'{actual_bytes[i]:02x}'
                token = f"{eh}/{ah}"
                if i in mis_set:
                    token = f"{red}{token}{reset}"
                elif i in ign_set:
                    token = f"{yellow}{token}{reset}"
                parts.append(token)
            rows.append(f"{base:04x}: " + ' '.join(parts))
        where = ','.join(str(i) for i in mismatches)
        return False, header + "\n" + "Mismatches at byte(s) " + where + ":\n" + "\n".join(rows)

    # Success
    if cfg:
        return True, f"{length}/{length} bytes match"
    return True, f"{length}/{length} bytes match"


def run_single_test(program, verbose=False, keep_hex=False, cycles=1_000_000):
    """
    Run a single test program.

    Returns:
        Tuple of (pass/fail bool, message string)
    """
    bin_file = TESTS_DIR / f"{program}.bin"
    hex_file = TESTS_DIR / f"{program}.hex"
    res_file = TESTS_DIR / f"res_{program}.bin"

    if not bin_file.exists():
        return False, f"Test program {bin_file} not found"

    try:
        # Convert bin to hex
        if verbose:
            print(f"  Converting {bin_file} to {hex_file}...")
        # Convert; for specific programs we may apply a small patch
        if program == 'div':
            # Patch bytes at offsets 1016 and 1017 to 0x90 0x90
            convert_bin_to_hex(
                bin_file,
                hex_file,
                verbose=verbose,
                patch_addr=0x1016,
                patch_bytes="90 90",
            )
        elif program == 'segpr':
            # Patch bytes at 0x110E with three NOPs
            convert_bin_to_hex(
                bin_file,
                hex_file,
                verbose=verbose,
                patch_addr=0x110E,
                patch_bytes="90 90 90",
            )
        else:
            convert_bin_to_hex(
                bin_file,
                hex_file,
                verbose=verbose,
            )

        # Determine result length from expected file (use full length, no trimming)
        if res_file.exists():
            result_len = len(res_file.read_bytes())
        else:
            result_len = 16  # Default

        # Run test
        if verbose:
            print(f"  Running simulation (max {cycles} cycles, checking {result_len} bytes @0; reset CS=F000 IP=FFF0)...")

        stdout, stderr, returncode = run_test_program(hex_file, max_cycles=cycles, result_len=result_len, verbose=verbose)

        if returncode != 0 and verbose:
            print(f"  Testbench returned code {returncode}")

        # if verbose:
        #     print(f"  Output: {stdout}")

        # Extract memory
        actual_bytes = extract_memory_from_output(stdout)

        if verbose:
            print(f"  Extracted {len(actual_bytes)} bytes from output")
            if actual_bytes:
                print(f"    Memory @0: {' '.join(f'{b:02x}' for b in actual_bytes[:16])}")

        # Validate
        passed, message = validate_result(program, actual_bytes, res_file)

        # Cleanup hex file unless keeping
        if not keep_hex and hex_file.exists():
            hex_file.unlink()

        return passed, message

    except subprocess.TimeoutExpired:
        return False, "Simulation timeout (30s)"
    except Exception as e:
        return False, f"Error: {e}"


def main():
    parser = argparse.ArgumentParser(
        description='Run z8086 multi-instruction test programs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python run_programs.py                    # Run all tests
  python run_programs.py control add mul    # Run specific tests
  python run_programs.py -v                 # Verbose output
  python run_programs.py --keep-hex         # Keep .hex files after testing
        '''
    )

    parser.add_argument('programs', nargs='*',
                        help='Specific programs to test (default: all)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose output')
    parser.add_argument('--keep-hex', action='store_true',
                        help='Keep .hex files after testing')
    parser.add_argument('-c', '--cycles', type=int, default=1_000_000,
                        help='Maximum simulation cycles per program (default: 10,000,000)')
    parser.add_argument('--list', action='store_true',
                        help='List available test programs')

    args = parser.parse_args()

    # List programs if requested
    if args.list:
        print("Available test programs:")
        for prog in TEST_PROGRAMS:
            bin_file = TESTS_DIR / f"{prog}.bin"
            status = "✓" if bin_file.exists() else "✗"
            print(f"  {status} {prog}")
        return

    # Check testbench exists
    if not VERILATOR_EXE.exists():
        print(f"Error: Testbench {VERILATOR_EXE} not found", file=sys.stderr)
        print("Run 'make build' first", file=sys.stderr)
        sys.exit(1)

    # Determine which programs to run
    if args.programs:
        programs_to_run = args.programs
    else:
        programs_to_run = TEST_PROGRAMS

    # Run tests
    results = {}
    passed_count = 0
    failed_count = 0

    print(f"Running {len(programs_to_run)} test program(s)...\n")

    for program in programs_to_run:
        if args.verbose:
            print(f"Testing {program}.bin:")

        passed, message = run_single_test(program, verbose=args.verbose, keep_hex=args.keep_hex, cycles=args.cycles)

        results[program] = (passed, message)

        if passed:
            passed_count += 1
            status = "PASS"
            color = "\033[92m"  # Green
        else:
            failed_count += 1
            status = "FAIL"
            color = "\033[91m"  # Red

        reset = "\033[0m"

        if args.verbose:
            print(f"  {color}{status}{reset}: {message}\n")
        else:
            print(f"{color}{status}{reset} {program:15s} - {message}")

    # Summary
    print(f"\n{'='*60}")
    total = passed_count + failed_count
    percentage = (passed_count * 100 / total) if total > 0 else 0
    print(f"Summary: {passed_count}/{total} programs passed ({percentage:.1f}%)")

    if failed_count > 0:
        print(f"\nFailed programs:")
        for program, (passed, message) in results.items():
            if not passed:
                if args.verbose:
                    print(f"  - {program}: {message}")
                else:
                    print(f"  - {program}")

    sys.exit(0 if failed_count == 0 else 1)


if __name__ == '__main__':
    # Initialize flags config before running
    try:
        init_flags_config()
    except Exception:
        # Safe to continue without flags config
        pass
    main()
