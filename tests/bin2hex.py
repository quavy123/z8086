#!/usr/bin/env python3
"""
Binary to Hex Converter for z8086 Test Programs

Converts 64KB .bin memory images to sparse .hex format for Verilator testbench.
The .hex format uses @address notation to specify non-contiguous memory regions.
"""

import sys
import argparse
from pathlib import Path


def find_nonzero_regions(data, min_gap=16):
    """
    Find contiguous regions of non-zero bytes in binary data.

    Args:
        data: Binary data as bytes
        min_gap: Minimum gap between regions to split them (default 16 bytes)

    Returns:
        List of (start_addr, end_addr) tuples for non-zero regions
    """
    regions = []
    in_region = False
    start = 0
    last_nonzero = 0

    for i, byte in enumerate(data):
        if byte != 0:
            if not in_region:
                # Start new region
                start = i
                in_region = True
            last_nonzero = i
        elif in_region and (i - last_nonzero) >= min_gap:
            # End region if we've seen enough zeros
            regions.append((start, last_nonzero + 1))
            in_region = False

    # Close final region if still open
    if in_region:
        regions.append((start, last_nonzero + 1))

    return regions


def _parse_patch_bytes(s: str):
    """Parse a string of hex bytes into a list of ints.

    Accepts formats like:
      "EA 00 00 00 F0"
      "EA,00,00,00,F0"
      "EA000000F0"
    """
    if s is None:
        return None
    # Remove commas and whitespace, but preserve separation if present
    cleaned = s.replace(',', ' ').strip()
    if ' ' in cleaned:
        tokens = [t for t in cleaned.split() if t]
        return [int(t, 16) for t in tokens]
    # No separators; treat as contiguous hex pairs
    if len(cleaned) % 2 != 0:
        raise ValueError("patch-bytes must contain an even number of hex digits")
    return [int(cleaned[i:i+2], 16) for i in range(0, len(cleaned), 2)]


def bin2hex(bin_file, hex_file, verbose=False, base=0xF0000, *, patch_addr=None, patch_bytes=None):
    """
    Convert binary memory image to sparse hex format.

    Args:
        bin_file: Path to input .bin file (64KB memory image)
        hex_file: Path to output .hex file
        verbose: Print conversion statistics
    """
    # Read binary file
    data = bytearray(bin_file.read_bytes())

    if len(data) != 65536:
        print(f"Warning: Binary file is {len(data)} bytes, expected 65536 bytes")

    # Pad to 64KB if needed
    if len(data) < 65536:
        data = data + bytes(65536 - len(data))

    # Optional patching of the binary before conversion
    if patch_addr is not None and patch_bytes:
        if patch_addr < 0 or patch_addr + len(patch_bytes) > len(data):
            raise ValueError(f"Patch out of range: addr={patch_addr}, len={len(patch_bytes)} for image {len(data)} bytes")
        for i, b in enumerate(patch_bytes):
            data[patch_addr + i] = b & 0xFF

    # Find non-zero regions
    regions = find_nonzero_regions(data)

    if verbose:
        total_bytes = sum(end - start for start, end in regions)
        print(f"Found {len(regions)} non-zero region(s)")
        print(f"Total non-zero bytes: {total_bytes} / 65536 ({total_bytes*100/65536:.1f}%)")

    # Write sparse hex file
    with open(hex_file, 'w') as f:
        for start, end in regions:
            # Write address marker
            f.write(f'@{start+base:05x}\n')

            # Write hex bytes (one per line)
            for addr in range(start, end):
                f.write(f'{data[addr]:02x}\n')

            if verbose:
                print(f"  Region: @{start+base:05x} - @{end-1+base:05x} ({end-start} bytes)")

    if verbose:
        hex_size = hex_file.stat().st_size
        bin_size = bin_file.stat().st_size
        print(f"Output: {hex_file.name} ({hex_size} bytes, {hex_size*100/bin_size:.1f}% of original)")


def main():
    parser = argparse.ArgumentParser(
        description='Convert 64KB binary memory images to sparse hex format for z8086 testbench',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python bin2hex.py 80186_tests/add.bin
  python bin2hex.py 80186_tests/control.bin -o control.hex
  python bin2hex.py 80186_tests/mul.bin -v
        '''
    )

    parser.add_argument('bin_file', type=Path,
                        help='Input .bin file (64KB memory image)')
    parser.add_argument('-o', '--output', type=Path,
                        help='Output .hex file (default: same name as input with .hex extension)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Print conversion statistics')
    parser.add_argument('-b', '--base', type=int, 
                        help='Base address for the hex file (default: 0xF0000)', default=0xF0000)
    parser.add_argument('--patch-addr', type=int,
                        help='Starting address (offset in 64KB image) to patch before conversion')
    parser.add_argument('--patch-bytes', type=str,
                        help='Hex bytes to write at patch-addr (e.g. "EA 00 00 00 F0")')

    args = parser.parse_args()

    # Check input file exists
    if not args.bin_file.exists():
        print(f"Error: Input file '{args.bin_file}' not found", file=sys.stderr)
        sys.exit(1)

    # Determine output filename
    if args.output:
        hex_file = args.output
    else:
        hex_file = args.bin_file.with_suffix('.hex')

    # Convert
    try:
        patch = _parse_patch_bytes(args.patch_bytes) if args.patch_bytes else None
        bin2hex(args.bin_file, hex_file, args.verbose, args.base,
                patch_addr=args.patch_addr, patch_bytes=patch)
        if not args.verbose:
            print(f"Converted {args.bin_file} -> {hex_file}")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
