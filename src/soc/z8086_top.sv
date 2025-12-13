// z8086 top module
module z8086_top (
    input clk50,
    input s0,
    input s1,
    output reg [7:0] led,
    // PMOD TFT LCD SPI pins
    output reg TFT_SPI_CLK,
    output reg TFT_RS,
    output reg TFT_SPI_MOSI,
    output reg TFT_SPI_CS
);

reg cpu_reset = 1'b0;
reg [19:0] reset_cnt = 20'hFFFFF;

always @(posedge clk50) begin
    reset_cnt <= reset_cnt == 20'h0 ? 20'h0 : reset_cnt - 20'h1;
    if (reset_cnt == 0 && ~s0) begin
        cpu_reset <= 1'b1;
    end
end

wire [19:0] addr;
wire [15:0] din;
wire [15:0] dout;
wire wr;
wire rd;
wire io;
wire word;
reg ready;
reg [1:0] state;

// -------------------------------------------------------------------------
// Simple SPI engine for TFT PMOD (ILI9341) via I/O port 7
// - Write 16-bit to port 7: dout[7:0]=data byte, dout[8]=D/C (RS)
// - Read port 7: returns {15'b0, ready} where ready=1 when idle
// SPI mode 0 (CPOL=0, CPHA=0)
// -------------------------------------------------------------------------
reg        spi_busy = 1'b0;
reg [7:0]  spi_shreg = 8'd0;
reg [3:0]  spi_bitcnt = 4'd0;
reg [7:0]  spi_div = 8'd0;
reg        spi_start = 1'b0;
reg        spi_start_dc = 1'b0;
reg [7:0]  spi_start_byte = 8'd0;
wire       spi_tick = (spi_div == 8'd0);

// target SCK ~1 MHz from 50 MHz: reload value 25-1 = 24 (half period)
localparam [7:0] SPI_DIV_RELOAD = 8'd4;

// divider updates when busy
always @(posedge clk50) begin
    if (spi_busy) begin
        spi_div <= spi_tick ? SPI_DIV_RELOAD : (spi_div - 8'd1);
    end else begin
        spi_div <= SPI_DIV_RELOAD;
    end
end

// SPI bit shifting and pin driving
always @(posedge clk50) begin
    if (!spi_busy) begin
        TFT_SPI_CLK <= 1'b0;     // idle low
        TFT_SPI_CS  <= 1'b1;     // deassert CS when idle
        if (spi_start) begin
            // launch new transfer
            spi_shreg     <= spi_start_byte;
            spi_bitcnt    <= 4'd8;
            TFT_RS        <= spi_start_dc;
            TFT_SPI_CS    <= 1'b0;            // assert CS
            TFT_SPI_MOSI  <= spi_start_byte[7];
            spi_busy      <= 1'b1;
        end
    end else if (spi_tick) begin
        TFT_SPI_CLK <= ~TFT_SPI_CLK; // toggle clock when busy
        if (TFT_SPI_CLK) begin
            // falling edge: shift data and present next MOSI
            spi_shreg     <= {spi_shreg[6:0], 1'b0};
            spi_bitcnt    <= spi_bitcnt - 4'd1;
            TFT_SPI_MOSI  <= spi_shreg[6];
            if (spi_bitcnt == 4'd1) begin
                // finished after shifting last bit
                spi_busy    <= 1'b0;
                TFT_SPI_CS  <= 1'b1; // deassert CS when idle
            end
        end
        // on rising edge: data is stable already (CPHA=0)
    end
end

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

reg [15:0] din_r;
wire [7:0] q;
reg q_out_lo, q_out_hi;
reg wr_2nd;
spram #(
    .width(8),
    .widthad(17),
    .init_file("blinky.hex")
    // .init_file("lcd_bars.hex")
    // .init_file("lcd_shapes.hex")
) ram (
    .clk(clk50),
    .wraddress(state == 2'd2 ? addr[16:0]+17'd1 : addr[16:0]),
    .wren(wr & ~io | wr_2nd),
    .data(state == 2'd2 ? dout[15:8] : dout[7:0]),
    .rdaddress(state == 2'd1 ? addr[16:0]+17'd1 : addr[16:0]),
    .q(q)
);
assign din = q_out_hi ? {q, din_r[7:0]} : 
             q_out_lo ? {din_r[15:8], q} : din_r;

// memory and io access
always @(posedge clk50) begin
    led[7] <= ~cpu_reset;

    ready <= 1'b0;
    q_out_lo <= 1'b0;
    q_out_hi <= 1'b0;
    wr_2nd <= 1'b0;
    case (state)
    2'd0: begin
        if (rd & ~io) begin
            q_out_lo <= 1'b1;
            state <= word;
            if (~word) ready <= 1'b1;
        end else if (wr & ~io) begin
            state <= word ? 2'd2 : 2'd0;
            if (~word) ready <= 1'b1;
            if (word) wr_2nd <= 1'b1;
        end
        // default: one-cycle start pulse low
        spi_start <= 1'b0;
        if ((rd | wr) & io) begin
            case (addr[7:0])
            8'h5: begin
                if (rd) din_r <= led;
                if (wr) led[6:0] <= dout[6:0];
            end
            8'h6: begin   // port 6: buttons
                if (rd) din_r <= {~s1, ~s0};
            end
            8'h7: begin   // port 7: TFT SPI interface
                // Read: return ready (1=ready)
                if (rd) din_r <= {15'b0, ~spi_busy};
                // Write (word access): launch one byte when idle
                if (wr && word && ~spi_busy) begin
                    spi_start      <= 1'b1;
                    spi_start_dc   <= dout[8];
                    spi_start_byte <= dout[7:0];
                end
            end
            default: ;
            endcase
            ready <= 1'b1;
        end
    end
    2'd1: begin  // read 2nd byte
        q_out_hi <= 1'b1;
        state <= 2'd0;
        ready <= 1'b1;
    end
    2'd2: begin  // write 2nd byte
        state <= 2'd0;
        ready <= 1'b1;
    end
    endcase

    if (q_out_lo) din_r[7:0] <= q;
    if (q_out_hi) din_r[15:8] <= q;
end

endmodule
