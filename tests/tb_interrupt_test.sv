`timescale 1ns/1ns

// Simple interrupt test
// Tests both NMI and INTR functionality
module tb_interrupt_test;
  reg clk = 0;
  always #5 clk = ~clk; // 100 MHz

  reg reset_n = 0;
  wire [19:0] ad;
  wire [15:0] dout;
  wire wr, rd, io, word;
  reg [15:0] din_mem;
  wire [15:0] din;
  reg ready;
  reg intr = 0;
  reg nmi  = 0;
  wire inta;

  // ANSI color codes for pretty output
  localparam string C_RED   = "\033[31m";
  localparam string C_GRN   = "\033[32m";
  localparam string C_YEL   = "\033[33m";
  localparam string C_CYN   = "\033[36m";
  localparam string C_RST   = "\033[0m";

  // Waveform tracing
  initial begin
    if ($test$plusargs("trace")) begin
      $dumpfile("wave.fst");
      $dumpvars(0, tb_interrupt_test);
    end
  end

  // Instantiate CPU
  z8086 dut (
    .clk(clk), .reset_n(reset_n),
    .addr(ad), .din(din), .dout(dout),
    .wr(wr), .rd(rd), .io(io), .word(word), .ready(ready),
    .intr(intr), .nmi(nmi), .inta(inta)
  );

  // Simple memory model
  localparam MEM_BYTES = 1<<20;
  reg [7:0] mem [0:MEM_BYTES-1];

  reg rd_d, io_d;
  always @(posedge clk) rd_d <= rd;

  assign din = din_mem;

  // Memory behavior
  always @(posedge clk) begin
    ready <= 1'b0;
    din_mem <= 16'h0000;
    if (rd) begin
      rd_d <= 1'b1;
      io_d <= io;
    end
    if (rd_d) begin
      ready <= 1'b1;
      if (io_d)
        din_mem <= 16'hffff;
      else
        din_mem <= { mem[ad+1], mem[ad] };
      rd_d <= 1'b0;
    end

    // Writes
    if (wr) begin
      if (!io) begin
        mem[ad] <= dout[7:0];
        if (word) mem[ad+1] <= dout[15:8];
      end
      ready <= 1'b1;
    end

    // inta
    if (inta) begin
      din_mem <= 16'h0005;
      ready <= 1'b1;
    end
  end

  // Load test program
  initial begin
    integer i;
    for (i = 0; i < MEM_BYTES; i = i + 1) mem[i] = 8'h00;

    // IVT Entry for vector 2 (NMI) at 0x00008-0x0000B: IP=0x0200, CS=0x0000
    mem[20'h00008] = 8'h00;
    mem[20'h00009] = 8'h02;
    mem[20'h0000A] = 8'h00;
    mem[20'h0000B] = 8'h00;

    // IVT Entry for vector 5 (INTR) at 0x00014-0x00017: IP=0x0300, CS=0x0000
    mem[20'h00014] = 8'h00;
    mem[20'h00015] = 8'h03;
    mem[20'h00016] = 8'h00;
    mem[20'h00017] = 8'h00;

    // NMI ISR at 0x00200: MOV AX,0x1111; IRET
    mem[20'h00200] = 8'hB8; // MOV AX
    mem[20'h00201] = 8'h11;
    mem[20'h00202] = 8'h11;
    mem[20'h00203] = 8'hCF; // IRET

    // INTR ISR at 0x00300: MOV AX,0x2222; IRET
    mem[20'h00300] = 8'hB8; // MOV AX
    mem[20'h00301] = 8'h22;
    mem[20'h00302] = 8'h22;
    mem[20'h00303] = 8'hCF; // IRET

    // Main program at reset vector 0xFFFF0
    mem[20'hFFFF0] = 8'hFA; // CLI
    mem[20'hFFFF1] = 8'hB8; // MOV AX, 0x0000
    mem[20'hFFFF2] = 8'h00;
    mem[20'hFFFF3] = 8'h00;
    mem[20'hFFFF4] = 8'hBB; // MOV BX, 0x1000
    mem[20'hFFFF5] = 8'h00;
    mem[20'hFFFF6] = 8'h10;
    mem[20'hFFFF7] = 8'h8E; // MOV SS, BX
    mem[20'hFFFF8] = 8'hD3;
    mem[20'hFFFF9] = 8'hBC; // MOV SP, 0x0000
    mem[20'hFFFFA] = 8'h00;
    mem[20'hFFFFB] = 8'h00;
    mem[20'hFFFFC] = 8'hFB; // STI
    mem[20'hFFFD] = 8'h90; // NOP
    mem[20'hFFFE] = 8'hEB; // JMP -2 (infinite loop: JMP $)
    mem[20'hFFFFF] = 8'hFE; // -2 offset

    // Bring CPU out of reset
    repeat (5) @(posedge clk);
    reset_n = 1'b1;

    // Wait for CLI/STI sequence and NOPs to start
    repeat (500) @(posedge clk);

    // Test 1: Assert INTR (should be serviced after STI)
    $display("%s[TEST]%s Asserting INTR at time %0t", C_CYN, C_RST, $time);
    intr = 1'b1;

    // Wait for interrupt to be acknowledged
    fork
      begin
        @(posedge inta);
        $display("%s[TEST]%s INTA asserted at time %0t", C_CYN, C_RST, $time);
        $display("[DEBUG] At INTA: OPR=%04x, din=%04x, FC=%b, intr_pending=%b",
                 dut.OPR, din, dut.FC, dut.intr_pending);
        @(posedge clk);
        $display("[DEBUG] One cycle later: OPR=%04x, IP=%04x, CS=%04x", dut.OPR, dut.IP, dut.CS);
        @(negedge inta);
        intr = 1'b0;
        // Check where we're jumping to
        repeat (20) @(posedge clk);
        $display("[DEBUG] After ISR entry: IP=%04x, CS=%04x, AX=%04x", dut.IP, dut.CS, dut.AX);
      end
      begin
        repeat (200) @(posedge clk);
        $display("[DEBUG] Timeout waiting for INTA");
        $display("[DEBUG] intr_pending=%b, nmi_pending=%b, interrupt_request=%b",
                 dut.intr_pending, dut.nmi_pending, dut.interrupt_request);
        $display("[DEBUG] F[9]=%b (IF flag), FC=%b", dut.F[9], dut.FC);
        $display("[DEBUG] ROME=%b, RNI=%b, loader_state=%d", dut.ROME, dut.RNI, dut.loader_state);
        $display("[DEBUG] IP=%04x, AR=%03x, CR=%x", dut.IP, dut.AR, dut.CR);
        $display("[DEBUG] q_bus=%02x, not_halted=%b", dut.q_bus, dut.not_halted);
      end
    join_any
    disable fork;

    // Wait for ISR to complete
    repeat (50) @(posedge clk);

    // Check that AX was set by ISR
    if (dut.AX == 16'h2222) begin
      $display("%s[PASS]%s INTR test: AX = 0x%04x (expected 0x2222)", C_GRN, C_RST, dut.AX);
    end else begin
      $display("%s[FAIL]%s INTR test: AX = 0x%04x (expected 0x2222)", C_RED, C_RST, dut.AX);
    end

    // Test 2: Pulse NMI
    $display("%s[TEST]%s Pulsing NMI at time %0t", C_CYN, C_RST, $time);
    nmi = 1'b1;
    @(posedge clk);
    @(posedge clk);
    nmi = 1'b0;

    // Wait for NMI to be serviced
    repeat (100) @(posedge clk);

    // Check that AX was set by NMI ISR
    if (dut.AX == 16'h1111) begin
      $display("%s[PASS]%s NMI test: AX = 0x%04x (expected 0x1111)", C_GRN, C_RST, dut.AX);
    end else begin
      $display("%s[FAIL]%s NMI test: AX = 0x%04x (expected 0x1111)", C_RED, C_RST, dut.AX);
    end

    $display("%s[TEST]%s All interrupt tests completed. time=%0t", C_CYN, C_RST, $time);
    $finish;
  end

  // Timeout
  initial begin
    repeat (5000) @(posedge clk);
    $display("%s[ERROR]%s Test timeout", C_RED, C_RST);
    $finish;
  end

endmodule
