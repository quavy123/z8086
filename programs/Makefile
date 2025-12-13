NASM ?= nasm
PY   ?= python

SRCS := $(wildcard *.asm)
BINS := $(SRCS:.asm=.bin)
HEXS := $(SRCS:.asm=.hex)

# Select which program to install as program.hex for the SoC
PROGRAM ?= blinky

.PHONY: all clean list 

all: $(BINS) $(HEXS)

list:
	@echo "Sources: $(SRCS)"
	@echo "Binaries: $(BINS)"
	@echo "Hex: $(HEXS)"

# Assemble NASM flat binaries
%.bin: %.asm
	$(NASM) -f bin -o $@ $<

# Convert to contiguous 128KB hex using hexdump:
# - Prepend 64KB of zeros, then place program in the upper 64KB
# - Truncate/pad program to exactly 64KB so total is 128KB
%.hex: %.bin
	dd if=/dev/zero of="$@.img" bs=65536 count=2
	dd if="$<" of="$@.img" bs=65536 count=1 seek=1 conv=notrunc
	hexdump -v -e '1/1 "%02x\n"' "$@.img" > "$@"
	rm -f "$@.img"

clean:
	rm -f $(BINS) $(HEXS) 
