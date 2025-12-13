module spram #( 	
    parameter width = 1,
    parameter widthad = 1,
    parameter init_file = ""
)( 
    input                clk,
    
    input  [widthad-1:0] wraddress,
    input                wren,
    input    [width-1:0] data,
    
    input  [widthad-1:0] rdaddress,
    output   [width-1:0] q
);

   reg [width-1:0]   mem [(2**widthad)-1:0];
   reg [widthad-1:0] rdaddr;
   
   // Optional initialization from a hex file (one word per line)
   initial begin
      if (init_file != "") begin
//         $display("spram: loading %0s (%0d x %0d)", init_file, (1<<widthad), width);
         $readmemh(init_file, mem);
      end
   end
   
   always @(posedge clk) begin
       if (wren) mem[wraddress] <= data;
       rdaddr <= rdaddress;
   end
   
   assign q = mem[rdaddr];
   
endmodule
