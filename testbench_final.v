`timescale 1ns/1ns

module tb();
  reg clk, reset;
  wire [31:0] WriteData, DataAdr;
  wire MemWrite;
  top a(.clk(clk), .reset(reset),
        .WriteData(WriteData),
        .DataAdr(DataAdr),
        .MemWrite(MemWrite));
  always begin 
    //$display("RAM0=%h", a.dmem.RAM[0]);
    #10 
    clk = ~clk; 
  end
  initial begin
    clk = 1; reset = 1; #20
    reset = 0;
    #600
    //$display("RAM0=%h, RAM4=%h, RAM40=%h, RAM44=%h, RAM48=%h, RAM52=%h", a.dmem.RAM[0], a.dmem.RAM[1], a.dmem.RAM[10], a.dmem.RAM[11], a.dmem.RAM[12], a.dmem.RAM[13]);
    //$display("RAM96=%h RAM100=%h", a.dmem.RAM[24], a.dmem.RAM[25]);
    $finish;
  end
  initial begin
    $dumpfile("test.vcd");
    $dumpvars;
  end
endmodule
    
                      
  