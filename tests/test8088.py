#!/usr/bin/env python3
"""
Single-Instruction Test Runner for z8086

Runs "8088" single-instruction test set.
"""

import json
import subprocess as sp
from pathlib import Path
import tempfile
import argparse
import math

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / '04.z8086' / 'tests'
TB = TESTS / 'obj_dir' / 'Vtb_z8086'
TEST_DIR = TESTS / '8088'
VERBOSE = False

# 8086 opcode names (00-FF)
OPCODE_NAMES = [
"ADD Eb Gb", "ADD Ev Gv", "ADD Gb Eb", "ADD Gv Ev", "ADD AL Ib", "ADD AX Iv", "PUSH ES", "POP ES",
"OR Eb Gb", "OR Ev Gv", "OR Gb Eb", "OR Gv Ev", "OR AL Ib", "OR AX Iv", "PUSH CS", "X",

"ADC Eb Gb", "ADC Ev Gv", "ADC Gb Eb", "ADC Gv Ev", "ADC AL Ib", "ADC AX Iv", "PUSH SS", "POP SS",
"SBB Eb Gb", "SBB Ev Gv", "SBB Gb Eb", "SBB Gv Ev", "SBB AL Ib", "SBB AX Iv", "PUSH DS", "POP DS",

"AND Eb Gb", "AND Ev Gv", "AND Gb Eb", "AND Gv Ev", "AND AL Ib", "AND AX Iv", "ES:", "DAA",
"SUB Eb Gb", "SUB Ev Gv", "SUB Gb Eb", "SUB Gv Ev", "SUB AL Ib", "SUB AX Iv", "CS:", "DAS",

"XOR Eb Gb", "XOR Ev Gv", "XOR Gb Eb", "XOR Gv Ev", "XOR AL Ib", "XOR AX Iv", "SS:", "AAA",
"CMP Eb Gb", "CMP Ev Gv", "CMP Gb Eb", "CMP Gv Ev", "CMP AL Ib", "CMP AX Iv", "DS:", "AAS",

"INC AX", "INC CX", "INC DX", "INC BX", "INC SP", "INC BP", "INC SI", "INC DI",
"DEC AX", "DEC CX", "DEC DX", "DEC BX", "DEC SP", "DEC BP", "DEC SI", "DEC DI",

"PUSH AX", "PUSH CX", "PUSH DX", "PUSH BX", "PUSH SP", "PUSH BP", "PUSH SI", "PUSH DI",
"POP AX", "POP CX", "POP DX", "POP BX", "POP SP", "POP BP", "POP SI", "POP DI",

"X", "X", "X", "X", "X", "X", "X", "X",
"X", "X", "X", "X", "X", "X", "X", "X",

"JO Jb", "JNO Jb", "JB Jb", "JNB Jb", "JZ Jb", "JNZ Jb", "JBE Jb", "JA Jb",
"JS Jb", "JNS Jb", "JPE Jb", "JPO Jb", "JL Jb", "JGE Jb", "JLE Jb", "JG Jb",

"GRP1 Eb Ib", "GRP1 Ev Iv", "GRP1 Eb Ib", "GRP1 Ev Ib", "TEST Gb Eb", "TEST Gv Ev", "XCHG Gb Eb", "XCHG Gv Ev",
"MOV Eb Gb", "MOV Ev Gv", "MOV Gb Eb", "MOV Gv Ev", "MOV Ew Sw", "LEA Gv M", "MOV Sw Ew", "POP Ev",

"NOP", "XCHG CX AX", "XCHG DX AX", "XCHG BX AX", "XCHG SP AX", "XCHG BP AX", "XCHG SI AX", "XCHG DI AX",
"CBW", "CWD", "CALL Ap", "WAIT", "PUSHF", "POPF", "SAHF", "LAHF",

"MOV AL Ob", "MOV AX Ov", "MOV Ob AL", "MOV Ov AX", "MOVSB", "MOVSW", "CMPSB", "CMPSW",
"TEST AL Ib", "TEST AX Iv", "STOSB", "STOSW", "LODSB", "LODSW", "SCASB", "SCASW",

"MOV AL Ib", "MOV CL Ib", "MOV DL Ib", "MOV BL Ib", "MOV AH Ib", "MOV CH Ib", "MOV DH Ib", "MOV BH Ib",
"MOV AX Iv", "MOV CX Iv", "MOV DX Iv", "MOV BX Iv", "MOV SP Iv", "MOV BP Iv", "MOV SI Iv", "MOV DI Iv",

"X", "X", "RET Iw", "RET", "LES Gv Mp", "LDS Gv Mp", "MOV Eb Ib", "MOV Ev Iv",
"X", "X", "RETF Iw", "RETF", "INT 3", "INT Ib", "INTO", "IRET",

"GRP2 Eb 1", "GRP2 Ev 1", "GRP2 Eb CL", "GRP2 Ev CL", "AAM I0", "AAD I0", "X", "XLAT",
"ESC 0", "ESC 1", "ESC 2", "ESC 3", "ESC 4", "ESC 5", "ESC 6", "ESC 7",

"LOOPNZ Jb", "LOOPZ Jb", "LOOP Jb", "JCXZ Jb", "IN AL Ib", "IN AX Ib", "OUT Ib AL", "OUT Ib AX",
"CALL Jv", "JMP Jv", "JMP Ap", "JMP Jb", "IN AL DX", "IN AX DX", "OUT DX AL", "OUT DX AX",

"LOCK", "X", "REPNZ", "REPZ", "HLT", "CMC", "GRP3a Eb", "GRP3b Ev",
"CLC", "STC", "CLI", "STI", "CLD", "STD", "GRP4 Eb", "GRP5 Ev",
]

def build_if_needed():
    if TB.exists():
        return
    print('[build] verilating testbench...')
    sp.check_call(['make','-s','build'], cwd=str(TESTS))

def write_memhex(ram_pairs) -> str:
    # Use sparse $readmemh format with '@' address directives to avoid huge files
    lines = []
    cur = None
    for addr, val in sorted(ram_pairs, key=lambda p: p[0]):
        if cur != addr:
            lines.append(f"@{addr:05X}\n")
            cur = addr
        lines.append(f"{val:02X}\n")
        cur += 1
    tf = tempfile.NamedTemporaryFile('w', delete=False, suffix='.hex')
    tf.write(''.join(lines))
    tf.flush()
    tf.close()
    return tf.name

def run_case(case):
    name = case.get('name','')
    init = case['initial']
    fin  = case['final']
    cs = int(init['regs'].get('cs', 0))
    ip = int(init['regs'].get('ip', 0))
    ax = int(init['regs'].get('ax', 0))
    ram = init.get('ram', [])
    memhex = write_memhex(ram)

    nbytes = case.get('bytes')
    reads = 0
    if nbytes:
        reads = math.ceil(len(nbytes)/2)

    # Limit runtime to keep tests quick
    cycles = max(30000, reads * 500)
    cmd = [str(TB), f"+mem={memhex}", f"+cycles={cycles}"]
    for k, v in init['regs'].items():
        cmd.append(f"+{k.lower()}={v}")
    if reads:
        cmd.append(f"+reads={reads}")

    # Request outputs we need to check
    final_regs = fin.get('regs', {}) if fin else {}
    final_ram  = fin.get('ram', []) if fin else []
    for i, (addr, _) in enumerate(final_ram):
        cmd.append(f"+ram{i}={addr}")
    if VERBOSE:
        print("START REGS:", ' '.join(f"{k}:{v:04x}" for k,v in init['regs'].items()))
        print('END   REGS:', ' '.join(f"{k}:{v:04x}" for k,v in final_regs.items()))
        print("START RAM: ", ' '.join(f"{k:05x}:{v:02x}" for k,v in ram))
        print('END   RAM: ', ' '.join(f"{k:05x}:{v:02x}" for k,v in final_ram))

    if VERBOSE:
        print(f"Command: {' '.join(cmd)}")

    p = sp.run(cmd, cwd=str(TESTS), stdout=sp.PIPE, stderr=sp.STDOUT, text=True)
    out = p.stdout
    # print(f"Output: {out}")

    # Parse results
    regs = {}
    mem_results = {}
    import re
    for line in out.splitlines():
        if line.startswith('RESULT REG:'):
            vals = line.split(':')[1].strip().split(' ')
            for val in vals:
                m = val.split('=')
                if len(m) == 2:
                    regs[m[0].lower()] = int(m[1], 16) if m[1].startswith('0x') else int(m[1])
        if line.startswith('RESULT MEM:'):
            # print(line)
            m = line.split(':')[1].strip().split('=')
            if len(m) == 2:
                mem_results[int(m[0][1:])] = int(m[1])
    q_len = regs.get('q_len')
    q_consumed = regs.get('q_consumed')
    regs['ip'] = regs['ip'] - q_len - q_consumed
    # print(f"Adjusted IP: {regs['ip']}, Q_LEN: {q_len}")

    # For 8086, certain flags are undefined for decimal adjust instructions
    # DAA/DAS: OF is undefined
    # AAA/AAS: OF, SF, ZF, PF are undefined
    instr = name.strip().lower()
    flag_mask = 0xFFFF  # default: check all flags

    is_rotate = any(instr.startswith(x) for x in ["rol", "ror", "rcl", "rcr"])
    is_shift  = any(instr.startswith(x) for x in ["shl", "sal", "shr", "sar"])

    if instr in {"daa", "das"}:
        # Mask out OF (bit 11)
        flag_mask = ~0x0800
    elif instr in {"aaa", "aas"}:
        # Mask out OF (bit 11), SF (bit 7), ZF (bit 6), PF (bit 2)
        flag_mask = ~(0x0800 | 0x0080 | 0x0040 | 0x0004)
    elif instr.startswith("mul") or instr.startswith("imul"):
        # Mask out SF, ZF, AF, PF
        print(f"MUL/IMUL: {regs['ax']} * {regs['bx']} = {regs['dx']}:{regs['ax']}")
        flag_mask = ~(0x0080 | 0x0040 | 0x0010 | 0x0004)
    elif instr.startswith("aam"):
        # Mask out OF, AF, CF
        flag_mask = ~(0x0800 | 0x0010 | 0x0001)
    elif is_rotate or is_shift:

        if is_rotate:
            # Only check OF, CF, SF, ZF, AF, PF for rotates
            flag_mask &= (0x0800 | 0x0001 | 0x0080 | 0x0040 | 0x0010 | 0x0004)

        # Ignore AF for all shift
        if is_shift:
            flag_mask &= ~0x0010

        # Determine if multi-bit shift
        prefixes = {0x26, 0x2E, 0x36, 0x3E, 0xF0, 0xF2, 0xF3}
        opcode = None
        for b in case.get('bytes', []):
            if b not in prefixes:
                opcode = b
                break
        
        count = 1
        # D2/D3 are variable shift/rotate
        if opcode in [0xD2, 0xD3]:
            cx = init['regs'].get('cx', 0)
            count = cx & 0xFF
            
        if count != 1:
            # Ignore OF for multi-bit
            flag_mask &= ~0x0800

    ok = True
    for reg, val in final_regs.items():
        reg = reg.lower()
        got_val = regs.get(reg)
        if VERBOSE:
            if got_val is not None:
                print(f"Checking {reg} = {val:04x} (got {got_val:04x})")
            else:
                print(f"Checking {reg} = {val:04x} (got None)")
        if reg == 'flags' and flag_mask != 0xFFFF and got_val is not None:
            # Apply mask to ignore undefined flags
            exp = int(val) & flag_mask
            got = int(got_val) & flag_mask
            ok = ok and (got == exp)
        else:
            ok = ok and (regs.get(reg) == int(val))
    if final_ram:
        for addr, val in final_ram:
            v = mem_results.get(int(addr))
            if v is None:
                print(f"\033[91mRAM {addr:05x} not found in results\033[0m")
                ok = False
            else:
                if v != int(val):
                    print(f"\033[91mChecking RAM {addr:05x} = {val:02x} (got {v:02x})\033[0m")
                ok = ok and (v == int(val))

    return ok, out

def main():
    global VERBOSE
    ap = argparse.ArgumentParser(description='Run subset of 8088 tests on z8086 single-file core')
    ap.add_argument('--file', '-f', help='Run only this JSON test file (basename or path)')
    ap.add_argument('--idx', '-i', type=int, help='Run only this case index within the JSON file')
    ap.add_argument('--limit', '-n', type=int, default=10, help='Max cases per file (default: 10)')
    ap.add_argument('--no-color', action='store_true', help='Disable ANSI colors in grid')
    ap.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    args = ap.parse_args()
    VERBOSE = args.verbose
    use_color = not args.no_color

    build_if_needed()

    # Resolve test files
    if args.file:
        jf = Path(args.file)
        if not jf.suffix:
            jf = TEST_DIR / (jf.name + '.json') if not (TEST_DIR / jf.name).suffix else (TEST_DIR / jf.name)
        if not jf.exists():
            # Try within TEST_DIR
            cand = TEST_DIR / jf.name
            if cand.exists():
                jf = cand
            else:
                raise SystemExit(f"Test file not found: {args.file}")
        test_files = [jf]
    else:
        test_files = sorted(TEST_DIR.glob('*.json'))

    total = 0
    passed = 0
    agg = {}
    groups = {}
    for jf in test_files:
        data = json.loads(jf.read_text())
        indices = range(len(data)) if args.idx is None else [args.idx]
        count = 0
        for i in indices:
            if i < 0 or i >= len(data):
                continue
            if args.idx is None and count >= args.limit:
                break
            case = data[i]
            total += 1
            ok, out = run_case(case)
            status_plain = 'ok' if ok else 'fail'
            if use_color:
                col = '\033[92m' if ok else '\033[91m'
                status = f"{col}{status_plain}\033[0m"
            else:
                status = status_plain
            print(f"=== {jf.name}[{case.get('idx', i)}] {case.get('name','')}: {status} ===")
            if ok: passed += 1
            if not ok or VERBOSE:
                print("BYTES:", ' '.join(f"{b:02x}" for b in case['bytes']), f"@{case['initial']['ram'][0][0]:05x}")
                tail = '\n'.join(out.splitlines()[-3:])
                if VERBOSE:
                    print(f"OUTPUT:\n{out}")
                else:
                    print(f"TAIL:\n{tail}")
            # aggregate by opcode file
            base = Path(jf).stem.upper()
            if '.' in base:
                op, sub = base.split('.', 1)
                groups.setdefault(op, {}).setdefault(sub, {'p':0,'t':0})
                groups[op][sub]['t'] += 1
                if ok: groups[op][sub]['p'] += 1
                agg.setdefault(op, {'p':0,'t':0})
                agg[op]['t'] += 1
                if ok: agg[op]['p'] += 1
            else:
                agg.setdefault(base, {'p':0,'t':0})
                agg[base]['t'] += 1
                if ok: agg[base]['p'] += 1
            count += 1
    # Show percentage of successful test cases instead of a static note
    pct = (passed / total * 100.0) if total else 0.0
    summary_line = f"\nSummary: {passed}/{total} ({pct:.1f}%)"
    print(summary_line)
    if args.file is None:
        print_grid(agg, groups, use_color=use_color)
    # Always write summary to result.txt as well
    try:
        result_path = Path(__file__).parent / 'result.txt'
        with open(result_path, 'a') as f:
            f.write(summary_line + "\n")
    except Exception as e:
        if VERBOSE:
            print(f"[warn] Failed to append summary to {result_path}: {e}")

def _cell3xW(p, t, opcode_name="", group_name="", width=6, use_color=True):
    """Return a 3-line cell of fixed width (default 6).
    Line1: passes like "5/10", Line2: first word of opcode, Line3: rest of opcode name.
    For groups: Line1: empty, Line2: group name, Line3: empty.
    """
    if group_name:
        # Group cell: only show name on middle line
        empty = " " * width
        return [empty, group_name[:width].ljust(width), empty]
    if t == 0 and not opcode_name:
        empty = " " * width
        return [empty, empty, empty]

    # Line 1: progress like "5/10"
    if t > 0:
        status_plain = f"{p}/{t}".ljust(width)
        if use_color:
            if p == t:
                col = '\033[92m'  # green - only 100% pass
            elif p / t >= 0.5:
                col = '\033[93m'  # yellow - 50%+ pass
            else:
                col = '\033[91m'  # red - less than 50%
            status = f"{col}{status_plain}\033[0m"
        else:
            status = status_plain
    else:
        status = " " * width

    # Lines 2 & 3: opcode name split by first space, left-aligned
    if opcode_name and opcode_name != "X":
        parts = opcode_name.split(' ', 1)
        line2 = parts[0][:width].ljust(width)
        line3 = parts[1][:width].ljust(width) if len(parts) > 1 else " " * width
    else:
        line2 = " " * width
        line3 = " " * width

    return [status, line2, line3]

def print_grid(agg, groups, use_color=True):
    GROUPS = {"80","81","82","83","D0","D1","D2","D3","F6","F7","FE","FF"}
    W = 6  # Width for opcode names

    # Map opcode to group label
    OPCODE_TO_GROUP = {
        "80": "GRP1", "81": "GRP1", "82": "GRP1", "83": "GRP1",
        "D0": "GRP2", "D1": "GRP2", "D2": "GRP2", "D3": "GRP2",
        "F6": "GRP3a", "F7": "GRP3b",
        "FE": "GRP4", "FF": "GRP5",
    }

    # Group instruction names by ModR/M bits 3-5
    GROUP_NAMES = {
        "80": ["ADD Eb,Ib", "OR Eb,Gb", "ADC Eb,Gb", "SBB Eb,Gb", "AND Eb,Gb", "SUB Eb,Gb", "XOR Eb,Gb", "CMP Eb,Gb"],
        "81": ["ADD Ev,Iv", "OR Ev,Iv", "ADC Ev,Iv", "SBB Ev,Iv", "AND Ev,Iv", "SUB Ev,Iv", "XOR Ev,Iv", "CMP Ev,Iv"],
        "82": ["ADD Eb,Ib", "OR Eb,Ib", "ADC Eb,Ib", "SBB Eb,Ib", "AND Eb,Ib", "SUB Eb,Ib", "XOR Eb,Ib", "CMP Eb,Ib"],
        "83": ["ADD Ev,Ib", "OR Ev,Ib", "ADC Ev,Ib", "SBB Ev,Ib", "AND Ev,Ib", "SUB Ev,Ib", "XOR Ev,Ib", "CMP Ev,Ib"],
        "D0": ["ROL Eb,1", "ROR Eb,1", "RCL Eb,1", "RCR Eb,1", "SHL Eb,1", "SHR Eb,1", "", "SAR Eb,1"],
        "D1": ["ROL Ev,1", "ROR Ev,1", "RCL Ev,1", "RCR Ev,1", "SHL Ev,1", "SHR Ev,1", "", "SAR Ev,1"],
        "D2": ["ROL Eb,Cl", "ROR Eb,Cl", "RCL Eb,Cl", "RCR Eb,Cl", "SHL Eb,Cl", "SHR Eb,Cl", "", "SAR Eb,Cl"],
        "D3": ["ROL Ev,Cl", "ROR Ev,Cl", "RCL Ev,Cl", "RCR Ev,Cl", "SHL Ev,Cl", "SHR Ev,Cl", "", "SAR Ev,Cl"],
        "F6": ["TEST Eb", "", "NOT Eb", "NEG Eb", "MUL Eb", "IMUL Eb", "DIV Eb", "IDIV Eb"],
        "F7": ["TEST Ev", "", "NOT Ev", "NEG Ev", "MUL Ev", "IMUL Ev", "DIV Ev", "IDIV Ev"],
        "FE": ["INC Eb", "DEC Eb", "", "", "", "", "", ""],
        "FF": ["INC Ev", "DEC Ev", "CALL", "CALL Ep", "JMP", "JMP Ep", "PUSH", ""],
    }
    # Collect all output lines so we can both print and save
    out_lines = []
    # Top header with box drawing
    top = "    " + "┌" + "┬".join("─"*W for _ in range(16)) + "┐"
    hdr = "    │" + "│".join(f"{c:X}".center(W) for c in range(16)) + "│"
    sep = "    " + "├" + "┼".join("─"*W for _ in range(16)) + "┤"
    bot = "    " + "└" + "┴".join("─"*W for _ in range(16)) + "┘"
    out_lines.append("")
    out_lines.append("Grid (opcode names with pass/total):")
    out_lines.append(top)
    out_lines.append(hdr)
    out_lines.append(sep)
    for row in range(16):
        line1 = [f"{row:X}   ", "│"]
        line2 = ["    ", "│"]
        line3 = ["    ", "│"]
        for col in range(16):
            opcode = row * 16 + col
            op = f"{opcode:02X}"
            opcode_name = OPCODE_NAMES[opcode]
            if op in GROUPS:
                grp_name = OPCODE_TO_GROUP.get(op, "GRP")
                cell = _cell3xW(0, 0, "", grp_name, W, use_color)
            else:
                m = agg.get(op, {'p':0,'t':0})
                cell = _cell3xW(m['p'], m['t'], opcode_name, "", W, use_color)
            line1.append(cell[0]); line1.append("│")
            line2.append(cell[1]); line2.append("│")
            line3.append(cell[2]); line3.append("│")
        out_lines.append("".join(line1))
        out_lines.append("".join(line2))
        out_lines.append("".join(line3))
        if row != 15:
            out_lines.append(sep)
    out_lines.append(bot)
    if groups:
        out_lines.append("")
        out_lines.append("Group breakouts (subs 0..7, two groups per row):")
        # Build sorted list of group opcodes in pairs
        ops = sorted(groups.keys())
        # Single header (two blocks of 0..7)
        WG = 6  # Same width as main grid
        topG = "    " + "┌" + "┬".join("─"*WG for _ in range(16)) + "┐"
        hdrG = "    │" + "│".join((list("01234567")*2)[i].ljust(WG) for i in range(16)) + "│"
        sepG = "    " + "├" + "┼".join("─"*WG for _ in range(16)) + "┤"
        botG = "    " + "└" + "┴".join("─"*WG for _ in range(16)) + "┘"
        out_lines.append(topG)
        out_lines.append(hdrG)
        out_lines.append(sepG)
        for i in range(0, len(ops), 2):
            op1 = ops[i]
            op2 = ops[i+1] if i+1 < len(ops) else None
            # Build three rows (status, name1, name2) across 16 cells
            row1 = [f"{op1}  ", "│"]
            row2 = ["    ", "│"]
            row3 = ["    ", "│"]
            # left group
            for idx, s in enumerate(["0","1","2","3","4","5","6","7"]):
                m = groups.get(op1,{}).get(s, {'p':0,'t':0})
                instr_name = GROUP_NAMES.get(op1, [""] * 8)[idx]
                cell = _cell3xW(m['p'], m['t'], instr_name, False, WG, use_color)
                row1.append(cell[0]); row1.append("│")
                row2.append(cell[1]); row2.append("│")
                row3.append(cell[2]); row3.append("│")
            # right group (or blanks)
            for idx, s in enumerate(["0","1","2","3","4","5","6","7"]):
                if op2 is not None:
                    m = groups.get(op2,{}).get(s, {'p':0,'t':0})
                    instr_name = GROUP_NAMES.get(op2, [""] * 8)[idx]
                    cell = _cell3xW(m['p'], m['t'], instr_name, False, WG, use_color)
                else:
                    cell = _cell3xW(0, 0, "", False, WG, use_color)
                row1.append(cell[0]); row1.append("│")
                row2.append(cell[1]); row2.append("│")
                row3.append(cell[2]); row3.append("│")
            out_lines.append("".join(row1))
            out_lines.append("".join(row2))
            out_lines.append("".join(row3))
            if i+2 < len(ops):
                out_lines.append(sepG)
        out_lines.append(botG)

    # Print to console
    for l in out_lines:
        print(l)
    # Also save to result.txt next to this script
    result_path = Path(__file__).parent / 'result.txt'
    try:
        result_path.write_text("\n".join(out_lines) + "\n")
    except Exception as e:
        # Non-fatal: still show the grid even if saving fails
        if VERBOSE:
            print(f"[warn] Failed to write {result_path}: {e}")

if __name__ == '__main__':
    main()
