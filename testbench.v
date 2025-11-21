`timescale 1ns/1ns

module tb();
  reg clk, reset;
  wire [31:0] WriteData, DataAdr;
  wire MemWrite;
  top a(.clk(clk), .reset(reset),
        .WriteData(WriteData),
        .DataAdr(DataAdr),
        .MemWrite(MemWrite));
  always #5 clk = ~clk;
  initial begin
    clk = 1; reset = 1; #10
    reset = 0;
    #300
    $display("RAM96=%h, RAM100=%h", a.dmem.RAM[24], a.dmem.RAM[25]);
    $finish;
  end
  initial begin
    $dumpfile("test.vcd");
    $dumpvars;
  end
endmodule
    
                      
  