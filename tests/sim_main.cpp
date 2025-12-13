#include <verilated.h>
#include <verilated_fst_c.h>
#include "Vtb_z8086.h"

int main(int argc, char** argv) {
    auto contextp = new VerilatedContext;
    contextp->commandArgs(argc, argv);
    contextp->traceEverOn(true);
    auto top = new Vtb_z8086{contextp};
    // Tracing (FST)
    VerilatedFstC* tfp = new VerilatedFstC;
    top->trace(tfp, 99);
    tfp->open("wave.fst");

    // Parse a simple cycle limit from args: --cycles=N or +cycles=N or +max_cycles=N
    vluint64_t max_cycles = 100; // default
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        auto pos = a.find("=");
        if (pos != std::string::npos) {
            auto k = a.substr(0, pos);
            auto v = a.substr(pos + 1);
            if (k == "+cycles") {
                max_cycles = std::stoull(v);
                printf("Setting max_cycles to %llu\n", max_cycles);
            }
        }
    }

    vluint64_t cycles = 0;
    while (!contextp->gotFinish() && cycles < max_cycles) {
        top->eval();
        tfp->dump(contextp->time());
        contextp->timeInc(1);
        ++cycles;
    }

    tfp->close();
    delete tfp;

    delete top;
    delete contextp;
    return 0;
}
