#!/usr/bin/env python3
"""Tiny RV32I assembler emitting word-per-line $readmemh .hex firmware images
for stm32_ahb_soc_top (SRAM_HEX / FLASH_HEX parameters).

Images produced (run `python3 gen_fw.py` in this directory):
  fw_sram.hex  : boot from SRAM at 0x0000_0000 (BOOT_ADDR=0, the default).
                 Line 0 = address 0x0: reset trampoline jumping to main at
                 byte 0x2000 (word 0x800, i.e. line 2049).
  fw_flash.hex : identical word stream for XIP flash boot at 0x0002_0000
                 (BOOT_ADDR=32'h0002_0000). Line 0 of this file = address
                 0x0002_0000 (the flash model maps its line 0 to its base),
                 so its trampoline jumps to 0x0002_2000 instead.

Firmware behaviour (matches the tb_stm32_ahb_smoke PASS criterion):
  1. UART (0x8000_0000, 16550 map): LCR=0x83 (DLAB|8N1), DLL=0x36 (54),
     DLH=0, LCR=0x03  ->  8N1 at 100e6/(16*54) = ~115740 baud (8640 ns/bit).
     NOTE: this assembler parses every immediate as HEX (int(x, 16)).
  2. seed pattern 0x11,0x22,...,0x88 into SRAM 0x1000..0x101C (8 words).
  3. DMA ch0 (0x8000_6000): SRC=0x1000 DEST=0x3000 COUNT=8 CFG=1 (START);
     poll CFG until bit0 auto-clears (= transfer done).
  4. compare DEST region against SRC region word by word.
  5. all equal : print "PASS" via THR (polling LSR[5] THRE), wait TEMT, ebreak
     any differ: print "E", wait TEMT, ebreak.
The testbench decodes the UART characters: 4 chars == "PASS" -> SMOKE PASSED.

Mnemonics: lui addi andi li lw sw beq bne blt jalr ebreak
Labels "name:", origin "@0x2000 label:", "//" comments.
"""

import sys

# ---------------------------------------------------------------------------
# encoders
# ---------------------------------------------------------------------------
def r(name):
    regs = {f"x{i}": i for i in range(32)}
    regs.update({"zero": 0, "ra": 1, "sp": 2, "gp": 3, "tp": 4,
                 "t0": 5, "t1": 6, "t2": 7, "s0": 8, "fp": 8, "a0": 10})
    return regs[name]

def enc_i(f3, funct, rs1, rd, imm):
    assert -2048 <= imm < 2048, imm
    return (imm & 0xFFF) << 20 | rs1 << 15 | f3 << 12 | rd << 7 | funct

def enc_s(f3, rs2, rs1, imm):
    assert -2048 <= imm < 2048, imm
    return (((imm >> 5) & 0x7F) << 25 | rs2 << 20 | rs1 << 15 |
            f3 << 12 | (imm & 0x1F) << 7 | 0b0100011)

def enc_b(f3, rs2, rs1, imm):
    assert imm % 2 == 0 and -4096 <= imm < 4096, imm
    return (((imm >> 12) & 1) << 31 | ((imm >> 5) & 0x3F) << 25 | rs2 << 20 |
            rs1 << 15 | f3 << 12 | ((imm >> 1) & 0xF) << 8 |
            ((imm >> 11) & 1) << 7 | 0b1100011)

def enc_u(funct, rd, imm20):
    return (imm20 & 0xFFFFF) << 12 | rd << 7 | funct

LUI, JALR, LOAD, STORE, OPIMM = 0b0110111, 0b1100111, 0b0000011, 0b0100011, 0b0010011

def lui(rd, imm20):     return enc_u(LUI, rd, imm20)
def addi(rd, rs1, i):   return enc_i(0b000, OPIMM, rs1, rd, i)
def andi(rd, rs1, i):   return enc_i(0b111, OPIMM, rs1, rd, i)
def jalr(rd, i, rs1):   return enc_i(0b000, JALR, rs1, rd, i)
def sw(rs2, off, rs1):  return enc_s(0b010, rs2, rs1, off)
def lw(rd, off, rs1):   return enc_i(0b010, LOAD, rs1, rd, off)
def ebreak():           return 0x00100073

BR_F3 = {"beq": 0b000, "bne": 0b001, "blt": 0b100}

def li(rd, imm):
    assert 0 <= imm < (1 << 32), imm
    hi = (imm + 0x800) >> 12
    lo = imm - (hi << 12)              # [-2048, 2047]
    assert hi < (1 << 20), imm
    out = []
    if hi:
        out.append(lui(rd, hi))
    if lo or not out:
        out.append(addi(rd, rd if out else 0, lo))
    return out

# ---------------------------------------------------------------------------
# program
# ---------------------------------------------------------------------------
ASM = """
reset:
    lui  x1, TRAMPHI
    jalr x0, 0(x1)
    ebreak

@0x2000 main:
    li   x5, 0x80000000        // UART base
    li   x6, 0x83
    sw   x6, 12(x5)            // LCR = DLAB | 8N1
    li   x6, 0x36              // divisor 54 -> ~115740 baud at 100 MHz
    sw   x6, 0(x5)             // DLL
    li   x6, 0
    sw   x6, 4(x5)             // DLH
    li   x6, 3
    sw   x6, 12(x5)            // LCR = 8N1 (DLAB cleared)

    li   x10, 0x1000           // SRC pointer
    li   x12, 0x11             // pattern
    li   x13, 8                // word count
seed_loop:
    sw   x12, 0(x10)
    addi x10, x10, 4
    addi x12, x12, 0x11
    addi x13, x13, -1
    bne  x13, x0, seed_loop

    li   x5, 0x80006000        // DMA ch0 register base
    li   x6, 0x1000
    sw   x6, 0(x5)             // SRC
    li   x6, 0x3000
    sw   x6, 4(x5)             // DEST
    li   x6, 8
    sw   x6, 8(x5)             // COUNT (32-bit words)
    li   x6, 1
    sw   x6, 12(x5)            // CFG: START (auto-clears on done)
dma_poll:
    lw   x7, 12(x5)
    andi x7, x7, 1
    bne  x7, x0, dma_poll

    li   x5, 0x80000000        // UART base for printing
    li   x10, 0x1000
    li   x11, 0x3000
    li   x13, 8
verify_loop:
    lw   x15, 0(x10)
    lw   x16, 0(x11)
    bne  x15, x16, fail
    addi x10, x10, 4
    addi x11, x11, 4
    addi x13, x13, -1
    bne  x13, x0, verify_loop

putc_p:
    lw   x7, 20(x5)            // LSR
    andi x7, x7, 0x20          // THRE
    beq  x7, x0, putc_p
    li   x6, 0x50              // 'P'
    sw   x6, 0(x5)             // THR
putc_a:
    lw   x7, 20(x5)
    andi x7, x7, 0x20
    beq  x7, x0, putc_a
    li   x6, 0x41              // 'A'
    sw   x6, 0(x5)
putc_s0:
    lw   x7, 20(x5)
    andi x7, x7, 0x20
    beq  x7, x0, putc_s0
    li   x6, 0x53              // 'S'
    sw   x6, 0(x5)
putc_s1:
    lw   x7, 20(x5)
    andi x7, x7, 0x20
    beq  x7, x0, putc_s1
    li   x6, 0x53              // 'S'
    sw   x6, 0(x5)
temt_wait:
    lw   x7, 20(x5)
    andi x7, x7, 0x40          // TEMT
    beq  x7, x0, temt_wait
    ebreak

fail:
putc_e:
    lw   x7, 20(x5)
    andi x7, x7, 0x20
    beq  x7, x0, putc_e
    li   x6, 0x45              // 'E'
    sw   x6, 0(x5)
temt_fail:
    lw   x7, 20(x5)
    andi x7, x7, 0x40
    beq  x7, x0, temt_fail
    ebreak
"""

def assemble(tramphi):
    text = ASM.replace("TRAMPHI", "0x%X" % tramphi)
    words = {}
    pc = 0
    pending = []          # (pc, op, (rsa, rsb), label)
    labels = {}
    for line in text.splitlines():
        line = line.split("//")[0].strip()
        if not line:
            continue
        if line.startswith("@"):
            tgt, lab = line.split()
            pc = int(tgt.lstrip("@"), 16) // 4
            labels[lab.rstrip(":")] = pc
            continue
        if line.endswith(":"):
            labels[line[:-1]] = pc
            continue
        f = line.split(None, 1)
        op = f[0]
        ops = [t.strip() for t in f[1].split(",")] if len(f) > 1 else []
        if op in BR_F3:
            pending.append((pc, op, (ops[0], ops[1]), ops[2]))
            pc += 1
            continue
        if op == "li":
            for i in li(r(ops[0]), int(ops[1], 16)):
                words[pc] = i
                pc += 1
            continue
        if op == "lui":
            ins = lui(r(ops[0]), int(ops[1], 16))
        elif op == "addi":
            ins = addi(r(ops[0]), r(ops[1]), int(ops[2], 16))
        elif op == "andi":
            ins = andi(r(ops[0]), r(ops[1]), int(ops[2], 16))
        elif op == "jalr":
            m = ops[1].split("(")
            ins = jalr(r(ops[0]), int(m[0]), r(m[1][:-1]))
        elif op in ("lw", "sw"):
            m = ops[1].split("(")
            off, rs1 = int(m[0]), r(m[1][:-1])
            ins = lw(r(ops[0]), off, rs1) if op == "lw" else sw(r(ops[0]), off, rs1)
        elif op == "ebreak":
            ins = ebreak()
        else:
            sys.exit(f"unknown op {op}")
        words[pc] = ins
        pc += 1
    for bpc, op, (a, b), lab in pending:
        if lab not in labels:
            sys.exit(f"undefined label {lab}")
        imm = (labels[lab] - bpc) * 4
        words[bpc] = enc_b(BR_F3[op], r(b), r(a), imm)
    return words, labels

def dump(words, path, base_byte):
    top = max(words)
    with open(path, "w") as fh:
        for i in range(top + 1):
            fh.write(f"{words.get(i, 0):08x}\n")
    print(f"{path}: {top + 1} words (line 0 = address 0x{base_byte:08X})")

# ---------------------------------------------------------------------------
w, lab = assemble(0x2)                       # SRAM: trampoline -> 0x0000_2000
dump(w, "fw_sram.hex", 0x00000000)
wf, labf = assemble(0x22)                    # flash: trampoline -> 0x0002_2000
dump(wf, "fw_flash.hex", 0x00020000)

# ---------------------------------------------------------------------------
# self-checks (encoding + layout + branch targets)
# ---------------------------------------------------------------------------
def br_imm(i):
    imm = (((i >> 31) & 1) << 12 | ((i >> 7) & 1) << 11 |
           ((i >> 25) & 0x3F) << 5 | ((i >> 8) & 0xF) << 1)
    return imm - (1 << 13) if imm & (1 << 12) else imm

assert w[0] == 0x000020B7, hex(w[0])         # lui x1, 0x2   -> 0x0000_2000
assert wf[0] == 0x000220B7, hex(wf[0])       # lui x1, 0x22  -> 0x0002_2000
assert w[1] == 0x00008067 and wf[1] == w[1]  # jalr x0, 0(x1)
assert w[2] == 0x00100073                    # ebreak (safety)
assert lab["main"] == 0x800 and labf["main"] == 0x800
assert w[0x800] == 0x800002B7, hex(w[0x800]) # lui x5, 0x80000 (UART base)
assert w[0x803] == addi(6, 0, 54), hex(w[0x803])  # DLL divisor = 54 (0x36!)
assert all(w[k] == wf[k] for k in w if k != 0), "images differ beyond trampoline"
# backward branch offsets
assert br_imm(w[lab["seed_loop"] + 4]) == -16          # bne -> seed_loop
assert br_imm(w[lab["dma_poll"] + 2]) == -8            # bne -> dma_poll
assert br_imm(w[lab["putc_p"] + 2]) == -8              # beq -> putc_p
assert br_imm(w[lab["temt_wait"] + 2]) == -8           # beq -> temt_wait
assert br_imm(w[lab["putc_e"] + 2]) == -8              # beq -> putc_e
# forward branch: verify_loop's "bne x15,x16,fail" is its 3rd instruction
assert br_imm(w[lab["verify_loop"] + 2]) == (lab["fail"] - (lab["verify_loop"] + 2)) * 4
assert br_imm(w[lab["verify_loop"] + 6]) == -24        # bne -> verify_loop
# images end with ebreak
assert w[max(w)] == 0x00100073
print("encoding + layout self-check OK")
print(f"main at word 0x{lab['main']:X} (byte 0x{lab['main']*4:X}, line {lab['main']+1}), "
      f"last word 0x{max(w):X}")
