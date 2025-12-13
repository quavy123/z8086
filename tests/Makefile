TOP      ?= tb_z8086
VERILATOR ?= verilator

SRC_V = \
  ../src/z8086/z8086.sv \
  ../src/soc/spram.sv \
  ../src/z8086/alu.sv \
  tb_z8086.sv

SRC_V_INT = \
  ../src/z8086/z8086.sv \
  ../src/z8086/alu.s \
  tb_interrupt_test.sv

all: sim

build: obj_dir/V$(TOP)

sim: obj_dir/V$(TOP)
	./obj_dir/V$(TOP)

obj_dir/V$(TOP): $(SRC_V) sim_main.cpp
	$(VERILATOR) -cc --exe +incdir+../src --build -Wall -Wno-PINCONNECTEMPTY --timing --top-module $(TOP) \
	  -Wno-IMPLICITSTATIC -Wno-TIMESCALEMOD -Wno-UNDRIVEN -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-VARHIDDEN -Wno-PROCASSINIT -Wno-CASEINCOMPLETE \
	  --trace-fst -sv $(SRC_V) sim_main.cpp --error-limit 0

# Interrupt test targets
build-int: obj_dir/Vtb_interrupt_test

sim-int: obj_dir/Vtb_interrupt_test
	./obj_dir/Vtb_interrupt_test +trace

obj_dir/Vtb_interrupt_test: $(SRC_V_INT)
	$(VERILATOR) --binary --timing --top-module tb_interrupt_test -Wno-fatal \
	  -Wno-TIMESCALEMOD -Wno-UNDRIVEN -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-VARHIDDEN -Wno-PROCASSINIT -Wno-CASEINCOMPLETE \
	  --trace-fst -sv $(SRC_V_INT)

clean:
	rm -rf obj_dir *.vcd

.PHONY: all sim clean build build-int sim-int
