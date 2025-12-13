`timescale 1ns/1ns

module tb_z8086;
  // Clock/reset
  reg clk = 0;
  longint start_time;
  always #5 begin 
    clk = ~clk; // 100 MHz
  end
  reg reset_n = 0;

  int max_cycles;
  int stop_after_first = 1; // default: stop when first instruction finishes

  // CPU simplified bus
  wire [19:0] ad;
  wire [15:0] dout;
  wire        wr, rd, io;
  wire        word;
  reg  [15:0] din;
  reg         ready;
  reg         intr = 0;
  reg         nmi  = 0;
  wire        inta;
  wire        dbg_first_done;

  // Instantiate CPU
  z8086 dut (
    .clk(clk), .reset_n(reset_n),
    .addr(ad), .din(din), .dout(dout),
    .wr(wr), .rd(rd), .io(io), .word(word), .ready(ready),
    .intr(intr), .nmi(nmi), .inta(inta)
  );

  assign dbg_first_done = dut.dbg_first_done;

  // Simple 1MB byte-addressable memory model with 1-cycle read latency
  localparam MEM_BYTES = 1<<20; // 1MB
  reg [7:0] mem [0:MEM_BYTES-1];

  // For debug / pass criteria
  integer fetch_count = 0;
  reg hlt_fetched = 0;
  reg pass = 0;
  integer stop_reads = 0;

  // Optional checks (set via plusargs): +chk_ip +chk_ax, and +ram0=<addr>, +ram1=<addr>, ...
  int ram_addrs [0:1023];
  int ram_cnt = 0;

  // One-cycle handshake helper
  reg rd_d, io_d;
  always @(posedge clk) rd_d <= rd;

  // Memory behavior
  always @(posedge clk) begin
    ready <= 1'b0;
    din   <= 16'h0000;
    if (rd) begin
      // respond next cycle
      rd_d <= 1'b1;
      io_d <= io;
    end
    if (rd_d) begin
      ready <= 1'b1;
      if (io_d)
        din <= 16'hffff;
      else
        din <= { mem[ad+1], mem[ad] };
      $display("TB RESP ad=%05x bytes=%02x %02x din=%04x", ad, mem[ad+1], mem[ad], {mem[ad+1],mem[ad]});
      fetch_count <= fetch_count + 1;
      // if (mem[ad] == 8'hF4 || mem[ad+1] == 8'hF4) begin
      //   hlt_fetched <= 1'b1; // saw HLT opcode in stream
      // end
      rd_d <= 1'b0;
    end

    // Writes (if any)
    if (wr) begin
      if (!io) begin
        mem[ad] <= dout[7:0];
        if (word) mem[ad+1] <= dout[15:8];
      end else begin
        // For potential future I/O testing, detect OUT 0,AX to pass
        if (ad == 20'h00000 && dout == 16'h1234)
          pass <= 1'b1;
      end
      ready <= 1'b1; // complete immediately
    end
  end

  // Program at reset vector F000:FFF0 -> physical 0xFFFF0
string memfile;
int cs_arg, ds_arg, es_arg, ss_arg;
int ip_arg;
int ax_arg, cx_arg, dx_arg, bx_arg, sp_arg, bp_arg, si_arg, di_arg;
int f_arg;


initial begin
    integer i;
    int result_addr = 0;
    int result_len = 0;
    for (i = 0; i < MEM_BYTES; i = i + 1) mem[i] = 8'h00;
    if ($value$plusargs("mem=%s", memfile)) begin
      $readmemh(memfile, mem);
      $display("[TB] loaded mem image from %s", memfile);
    end else begin
      mem[20'hFFFF0] = 8'hB8; // MOV AX,imm16
      mem[20'hFFFF1] = 8'h34; // imm low
      mem[20'hFFFF2] = 8'h12; // imm high
      mem[20'hFFFF3] = 8'hF4; // HLT
    end

    if ($value$plusargs("cycles=%d", max_cycles)) begin
      $display("[TB] max_cycles=%0d", max_cycles);
    end else begin
      max_cycles = 100000;
    end

    // Control whether to stop after the first instruction (default 1)
    void'($value$plusargs("stop_after_first=%d", stop_after_first));
    $display("[TB] stop_after_first=%0d", stop_after_first);

    // Bring CPU out of reset
    repeat (5) @(posedge clk);

    // Support result_addr and result_len for easier test automation
    if ($value$plusargs("result_addr=%d", result_addr) &&
        $value$plusargs("result_len=%d", result_len)) begin
      for (int i = 0; i < result_len; i++) begin
        ram_addrs[ram_cnt] = result_addr + i;
        ram_cnt++;
      end
    end

    // Optional checks and overrides via plusargs (for test harness)
    for (int i = 0; i < 256; i++) begin
      int a;
      if ($value$plusargs($sformatf("ram%0d=%%d", i), a)) begin
        ram_addrs[ram_cnt] = a;
        ram_cnt++;
      end
    end

    if ($value$plusargs("cs=%d", cs_arg)) begin
      dut.CS = cs_arg[15:0];
      $display("[TB] CS=%04x", dut.CS);
    end
    if ($value$plusargs("ds=%d", ds_arg)) begin
      dut.DS = ds_arg[15:0];
    end
    if ($value$plusargs("es=%d", es_arg)) begin
      dut.ES = es_arg[15:0];
    end
    if ($value$plusargs("ss=%d", ss_arg)) begin
      dut.SS = ss_arg[15:0];
    end
    if ($value$plusargs("ip=%d", ip_arg)) begin
      dut.IP = ip_arg[15:0];
      $display("[TB] IP=%04x", dut.IP);
    end
    if ($value$plusargs("ax=%d", ax_arg)) begin
      dut.AX = ax_arg[15:0];
      $display("[TB] AX=%04x", dut.AX);
    end
    if ($value$plusargs("cx=%d", cx_arg)) begin
      dut.CX = cx_arg[15:0];
    end
    if ($value$plusargs("dx=%d", dx_arg)) begin
      dut.DX = dx_arg[15:0];
    end
    if ($value$plusargs("bx=%d", bx_arg)) begin
      dut.BX = bx_arg[15:0];
    end
    if ($value$plusargs("sp=%d", sp_arg)) begin
      dut.SP = sp_arg[15:0];
    end
    if ($value$plusargs("bp=%d", bp_arg)) begin
      dut.BP = bp_arg[15:0];
    end
    if ($value$plusargs("si=%d", si_arg)) begin
      dut.SI = si_arg[15:0];
    end
    if ($value$plusargs("di=%d", di_arg)) begin
      dut.DI = di_arg[15:0];
    end
    if ($value$plusargs("flags=%d", f_arg)) begin
      dut.F = f_arg[15:0];
    end

    reset_n = 1'b1;
end

  reg [3:0] check_cnt = 0;
  always @(posedge clk) begin
    // Trigger final checks on HLT (CPU halts), optional read-count stop, or near cycle limit
    if (((hlt_fetched || !dut.not_halted) || (stop_reads != 0 && fetch_count >= stop_reads)) && check_cnt == 0) begin
      check_cnt <= 4'd10; // allow EU to commit results
      $display("check_cnt=%0d", check_cnt);
    end else if (check_cnt != 0) begin
      check_cnt <= check_cnt - 1'b1;
    end else if ($time + 20 > max_cycles) begin
      $display("[TB] Timeout. fetch=%0d hlt_fetched=%0d pass=%0d", fetch_count, hlt_fetched, pass);
      check_cnt <= 1;      // force finish when cycles limit is about to be reached
    end
  end

  always @(posedge clk) begin
    // Only stop on first instruction completion if +stop_after_first != 0
    if (check_cnt == 1 || (stop_after_first != 0 && dut.dbg_first_done)) begin
      // collect last write in-flight
      logic last_write = dut.bus_pending & dut.bus_wr;
      logic [19:0] last_write_addr = dut.bus_addr;
      logic last_write_word = ~dut.L8_aux;
      logic [15:0] last_write_data = dut.OPR;
      int q_len = dut.q_wptr >= dut.q_rptr ? dut.q_wptr - dut.q_rptr : 3 + dut.q_wptr - dut.q_rptr;
      if (last_write) begin
        $display("LAST WRITE: @%0x=%0x", last_write_addr, last_write_data);
      end
      q_len = q_len * 2 - dut.q_hl;
      $display("RESULT REG: AX=0x%04x CX=0x%04x DX=0x%04x BX=0x%04x SP=0x%04x BP=0x%04x SI=0x%04x DI=0x%04x Flags=0x%02x IP=0x%04x CS=0x%04x DS=0x%04x ES=0x%04x SS=0x%04x Q_LEN=%0d Q_CONSUMED=%0d", 
                dut.AX, dut.CX, dut.DX, dut.BX, dut.SP, dut.BP, dut.SI, dut.DI, dut.F, dut.IP, dut.CS, dut.DS, dut.ES, dut.SS, q_len, dut.q_consumed);
      for (int i = 0; i < ram_cnt; i++) begin
        int a = ram_addrs[i];
        logic [7:0] mem_data = mem[a];
        // apply last write
        if (last_write && (last_write_addr == a || (last_write_addr+1 == a && last_write_word))) begin
          int off = a - last_write_addr;
          mem_data = last_write_data[off*8 +: 8];
          $display("LAST WRITE: @%0x=%0x", a, mem_data);
        end
        $display("RESULT MEM: @%0d=%0d", a, mem_data);
      end
      $finish;
      @(posedge clk);
    end
  end

endmodule
