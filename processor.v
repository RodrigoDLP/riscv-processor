module adder(input  [31:0] a, b, output [31:0] y);
  assign y = a + b; 
endmodule


module alu(input [31:0] a, b, input [2:0] alucontrol, output [31:0] result, output zero);
  wire [31:0] condinvb, sum; 
  wire v; // overflow 
  wire isAddSub; 
  reg [31:0] result_reg; 
  assign result = result_reg;
  assign condinvb = alucontrol[0] ? ~b : b; 
  assign sum = a + condinvb + alucontrol[0]; 
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0]; 
  always @* case (alucontrol)
      3'b000:  result_reg = sum; // add
      3'b001:  result_reg = sum; // subtract
      3'b010:  result_reg = a & b; // and
      3'b011:  result_reg = a | b; // or
      3'b100:  result_reg = a ^ b; // xor
      3'b101:  result_reg = sum[31] ^ v; // slt
      3'b110:  result_reg = a << b[4:0]; // sll
      3'b111:  result_reg = a >> b[4:0]; // srl
    default: result_reg = 32'bx;
    //FALTA: lui, sra, sltiu?, sltu?, bltu/bgeu, jalr, jal
    endcase
  assign zero = (result == 32'b0); 
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub; 
endmodule


module aludec(input opb5, input [2:0] funct3, input funct7b5, input [1:0] ALUOp, output [2:0] ALUControl);
  wire  RtypeSub; 
  reg [2:0] ALUControl_reg; 
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction
  assign ALUControl = ALUControl_reg;
  always @(*) case(ALUOp)
      2'b00: ALUControl_reg = 3'b000; // addition
      2'b01: ALUControl_reg = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl_reg = 3'b001; // sub
                          else          
                            ALUControl_reg = 3'b000; // add, addi
                 3'b010:    ALUControl_reg = 3'b101; // slt, slti
                 3'b110:    ALUControl_reg = 3'b011; // or, ori
                 3'b111:    ALUControl_reg = 3'b010; // and, andi
                 default:   ALUControl_reg = 3'bxxx; // ???
               endcase
    endcase
endmodule



module controller(input [6:0] op, input [2:0] funct3, input funct7b5,
                  output [1:0] ResultSrcD, output MemWriteD, JumpD, BranchD, ALUSrcD, RegWriteD,
                  output [1:0] ImmSrcD, output [2:0] ALUControlD);
  wire [1:0] ALUOp; 
  wire Branch; 
  maindec md(.op(op), .ResultSrc(ResultSrcD), .MemWrite(MemWriteD), .Branch(BranchD),
             .ALUSrc(ALUSrcD), .RegWrite(RegWriteD), .Jump(JumpD), .ImmSrc(ImmSrcD), .ALUOp(ALUOp)); 
  aludec ad(.opb5(op[5]), .funct3(funct3), .funct7b5(funct7b5), .ALUOp(ALUOp), .ALUControl(ALUControlD)); 
endmodule

module controller_registers(input clk, input reset,
                            input RegWriteD,
                            input [1:0] ResultSrcD,
                            input MemWriteD,
                            input JumpD,
                            input BranchD,
                            input [2:0] ALUControlD,
                            input ALUSrcD,
                            input [1:0] ImmSrc,
                            input ZeroE,
                            output PCSrcE, ResultSrcE0,
                            output [1:0] ImmSrcD,
                            output reg ALUSrcE,
                            output reg [2:0] ALUControlE,
                            output reg MemWriteM,
                            output reg [1:0] ResultSrcW,
                            output reg RegWriteW, RegWriteM);
  reg RegWriteE, MemWriteE, JumpE, BranchE;
  reg [1:0] ResultSrcE, ResultSrcM;
  assign ImmSrcD = ImmSrc;
  always @(posedge clk) begin
    if (reset) RegWriteE <= 0; else RegWriteE <= RegWriteD;
    if (reset) ResultSrcE <= 0; else ResultSrcE <= ResultSrcD;
    if (reset) MemWriteE <= 0; else MemWriteE <= MemWriteD;
    if (reset) JumpE <= 0; else JumpE <= JumpD;
    if (reset) BranchE <= 0; else BranchE <= BranchD;
    if (reset) ALUControlE <= 0; else ALUControlE <= ALUControlD;
    if (reset) ALUSrcE <= 0; else ALUSrcE <= ALUSrcD;
    if (reset) RegWriteM <= 0; else RegWriteM <= RegWriteE;
    if (reset) ResultSrcM <= 0; else ResultSrcM <= ResultSrcE;
    if (reset) MemWriteM <= 0; else MemWriteM <= MemWriteE;
    if (reset) RegWriteW <= 0; else RegWriteW <= RegWriteM;
    if (reset) ResultSrcW <= 0; else ResultSrcW <= ResultSrcM;
  end
  assign PCSrcE = (ZeroE & BranchE) | JumpE;
  assign ResultSrcE0 = ResultSrcE[0];
endmodule




//===========================================
/*module datapath(input  clk, reset, 
                input  [1:0]  ResultSrc, 
                input  PCSrc, ALUSrc, 
                input  RegWrite,
                input  [1:0]  ImmSrc, 
                input  [2:0]  ALUControl, 
                output Zero, 
                output [31:0] PC,
                input  [31:0] Instr, 
                output [31:0] ALUResult, WriteData, 
                input  [31:0] ReadData);*/
module datapath(input clk, reset,
                input RegWriteM, RegWriteW, MemWriteM, ALUSrcE, PCSrcE,
                input [1:0] ResultSrcW, ImmSrcD,
                input [2:0] ALUControlE,
                output ZeroE,
                output reg [31:0] PCF, WriteDataM, InstrD,
                output reg [31:0] ALUResultM,
                input [31:0] Instr, ReadData,
                input [1:0] ForwardAE, ForwardBE,
               input StallF, StallD, FlushD, FlushE, RSE0in,
                output reg [4:0] Rs1E, Rs2E, RdM, RdE, RdW,
                output wire [4:0] Rs1D, Rs2D,
               output RegWriteMout, RegWriteWout, RSE0out, PCSrcEout);
  localparam WIDTH = 32; // Define a local parameter for bus width
  /*wire [31:0] PCNext, PCPlus4, PCTarget; 
  wire [31:0] ImmExt; 
  wire [31:0] SrcA, SrcB; 
  wire [31:0] Result; */
  wire [31:0] PCFprime, PCPlus4F, RFRD1, RFRD2, ImmExtD, PCTargetE, SrcAE, SrcBE, WriteDataE, ALUResultE, ResultW;
    
  wire [4:0] RdD;
  
  reg [31:0] PCD, PCPlus4D, RD1E, RD2E, PCE, ImmExtE, PCPlus4E, PCPlus4M,
  ALUResultW, ReadDataW, PCPlus4W;
  //FETCH
  mux2 #(WIDTH) pcmux(.d0(PCPlus4F), .d1(PCTargetE), .s(PCSrcE), .y(PCFprime));
  adder pcadd4(.a(PCF), .b({WIDTH{1'b0}} + 4), .y(PCPlus4F));
  //DECODE
  regfile rf(.clk(clk), .reset(reset), .we3(RegWriteW), .a1(InstrD[19:15]), .a2(InstrD[24:20]), .a3(RdW),
             .wd3(ResultW), .rd1(RFRD1), .rd2(RFRD2));
  assign RdD = InstrD[11:7];
  assign Rs1D = InstrD[19:15];
  assign Rs2D = InstrD[24:20];
  extend ext(.instr(InstrD[31:7]), .immsrc(ImmSrcD), .immext(ImmExtD));
  //EXECUTE
  mux3 #(WIDTH) forwardamux(.d0(RD1E), .d1(ResultW), .d2(ALUResultM), .s(ForwardAE), .y(SrcAE));
  mux3 #(WIDTH) forwardbmux(.d0(RD2E), .d1(ResultW), .d2(ALUResultM), .s(ForwardBE), .y(WriteDataE));
  mux2 #(WIDTH) srcbmux(.d0(WriteDataE), .d1(ImmExtE), .s(ALUSrcE), .y(SrcBE));
  
  adder pcaddbranch(.a(PCE), .b(ImmExtE), .y(PCTargetE));
  alu alu(.a(SrcAE), .b(SrcBE), .alucontrol(ALUControlE), .result(ALUResultE), .zero(ZeroE));
  //MEMORY & WRITEBACK
  mux3 #(WIDTH) resultmux(.d0(ALUResultW), .d1(ReadDataW), .d2(PCPlus4W), .s(ResultSrcW), .y(ResultW));
  
  assign RegWriteMout = RegWriteM;
  assign RegWriteWout = RegWriteW;
  assign RSE0out = RSE0in;
  assign PCSrcEout = PCSrcE;
  
  always @(posedge clk) begin
    if (reset) PCF <= 0; else if (!StallF) PCF <= PCFprime;
    if (FlushD || reset) InstrD <= 0; else if (!StallD) InstrD <= Instr;
    if (FlushD || reset) PCD <= 0; else if (!StallD) PCD <= PCF;
    if (FlushD || reset) PCPlus4D <= 0; else if (!StallD) PCPlus4D <= PCPlus4F;
    if (FlushE || reset) RD1E <= 0; else RD1E <= RFRD1;
    if (FlushE || reset) RD2E <= 0; else RD2E <= RFRD2;
    if (FlushE || reset) PCE <= 0; else PCE <= PCD;
    if (FlushE || reset) Rs1E <= 0; else Rs1E <= Rs1D;
    if (FlushE || reset) Rs2E <= 0; else Rs2E <= Rs2D;
    if (FlushE || reset) RdE <= 0; else RdE <= RdD;
    if (FlushE || reset) ImmExtE <= 0; else ImmExtE <= ImmExtD;
    if (FlushE || reset) PCPlus4E <= 0; else PCPlus4E <= PCPlus4D;
    if (reset) ALUResultM <= 0; else ALUResultM <= ALUResultE;
    if (reset) WriteDataM <= 0; else WriteDataM <= WriteDataE;
    if (reset) RdM <= 0; else RdM <= RdE;
    if (reset) PCPlus4M <= 0; else PCPlus4M <= PCPlus4E;
    if (reset) ALUResultW <= 0; else ALUResultW <= ALUResultM;
    if (reset) ReadDataW <= 0; else ReadDataW <= ReadData;
    if (reset) RdW <= 0; else RdW <= RdM;
    if (reset) PCPlus4W <= 0; else PCPlus4W <= PCPlus4M;
  end

endmodule



module dmem(input  clk, we, input  [31:0] a, wd, output [31:0] rd);
  reg [31:0] RAM[63:0]; 
  assign rd = RAM[a[31:2]]; // word aligned
  always @(posedge clk) begin 
    if (we) RAM[a[31:2]] <= wd; 
  end
endmodule

module extend(input  [31:7] instr, input  [1:0]  immsrc, output [31:0] immext);
  reg [31:0] immext_reg; 
  assign immext = immext_reg;
  always @* case(immsrc) 
               // I-type 
      2'b00:   immext_reg = {{20{instr[31]}}, instr[31:20]}; 
               // S-type (stores)
      2'b01:   immext_reg = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   immext_reg = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   immext_reg = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
      default: immext_reg = 32'bx; // undefined
    endcase             
endmodule


module hazard_unit(input [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
                   input RegWriteM, RegWriteW, PCSrcE, ResultSrcE0,
                   output StallF, StallD, FlushD, FlushE,
                   output [1:0] ForwardAE, ForwardBE);
  assign ForwardAE[1] = (Rs1E == RdM) & RegWriteM & (Rs1E != 0);
  assign ForwardAE[0] = (Rs1E == RdW) & RegWriteW & (Rs1E != 0) & !((Rs1E == RdM) & RegWriteM);
  assign ForwardBE[1] = (Rs2E == RdM) & RegWriteM & (Rs2E != 0);
  assign ForwardBE[0] = (Rs2E == RdW) & RegWriteW & (Rs2E != 0) & !((Rs2E == RdM) & RegWriteM);
  wire lwStall;
  assign lwStall = ResultSrcE0 & ((Rs1D == RdE) | (Rs2D == RdE));
  assign StallF = lwStall;
  assign StallD = lwStall;
  assign FlushD = PCSrcE;
  assign FlushE = lwStall | PCSrcE;
endmodule


module imem(input  [31:0] a, output [31:0] rd);
  reg [31:0] RAM[63:0]; 
  initial begin
    integer j;
  for (j=0; j<64; j=j+1) RAM[j] = 32'b0;
    $readmemh("instructions.hex",RAM); 
  end
  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module maindec(input  [6:0] op, output [1:0] ResultSrc, output MemWrite, output Branch, ALUSrc,
               output RegWrite, Jump, output [1:0] ImmSrc, output [1:0] ALUOp); 
  reg [10:0] controls; 
  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls; 
  always @* case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls <= 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls <= 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls <= 11'b1_xx_0_0_00_0_10_0; // R-type
      7'b1100011: controls <= 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls <= 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls <= 11'b1_11_0_0_10_0_00_1; // jal
    7'b0000000: controls <= 11'b0; //safety measure for after reset
      default:    controls <= 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module mux2 (input  [WIDTH-1:0] d0, d1, input s, output [WIDTH-1:0] y);
  parameter WIDTH = 8;
  assign y = s ? d1 : d0; 
endmodule

module mux3 (input  [WIDTH-1:0] d0, d1, d2, input [1:0] s, output [WIDTH-1:0] y);
  parameter WIDTH = 8;
  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module regfile(input  clk, input reset, input  we3, input  [ 4:0] a1, a2, a3, 
               input  [31:0] wd3, output [31:0] rd1, rd2); 
  reg [31:0] rf[31:0]; 
  // write third port on rising edge of clock (A3/WD3/WE3)
  integer i;
  always @(negedge clk) begin
    if (reset) begin
      for (i = 0; i < 32; i = i + 1)
        rf[i] <= 32'b0;   // clear a todos los registros.
    end else if (we3) rf[a3] <= wd3; 
  end
  // read two ports combinationally (A1/RD1, A2/RD2)
  // register 0 hardwired to 0
  assign rd1 = (a1 != 0) ? rf[a1] : 0; 
  assign rd2 = (a2 != 0) ? rf[a2] : 0; 
endmodule

module riscvsingle(input  clk, reset, output [31:0] PC, input  [31:0] Instr, output MemWrite,
                   output [31:0] DataAdr, output [31:0] WriteData, input  [31:0] ReadData);
  wire [31:0] ALUResult; 
  wire ALUSrcD, RegWriteD, MemWriteD, BranchD, JumpD; 
  wire ZeroE, ALUSrcE, MemWriteM, RegWriteW, RegWriteM, ResultSrcE0; 
  wire [1:0] ResultSrcD, ImmSrcDin, ImmSrcD; 
  wire [1:0] ResultSrcW;
  wire [2:0] ALUControlD;
  wire [2:0] ALUControlE;
  wire PCSrcE; 
  wire [31:0] InstrD, PCF, WriteDataM, ALUResultM;
  wire [1:0] ForwardAE, ForwardBE;
  wire StallF, StallD, FlushD, FlushE, RegWriteMout, RegWriteWout, ResultSrcE0out, PCSrcEout;
  wire [4:0] Rs1E, Rs2E, RdM, RdE, RdW, Rs1D, Rs2D; 
  
  controller c(.op(InstrD[6:0]), .funct3(InstrD[14:12]), .funct7b5(InstrD[30]), .ResultSrcD(ResultSrcD), .MemWriteD(MemWriteD),
               .JumpD(JumpD), .BranchD(BranchD), .ALUSrcD(ALUSrcD), .RegWriteD(RegWriteD), .ImmSrcD(ImmSrcDin),
               .ALUControlD(ALUControlD));
  
  controller_registers cr(.clk(clk), .reset(reset), .RegWriteD(RegWriteD), .ResultSrcD(ResultSrcD), .MemWriteD(MemWriteD),
                          .JumpD(JumpD),
                          .BranchD(BranchD), .ALUControlD(ALUControlD), .ALUSrcD(ALUSrcD), .ImmSrc(ImmSrcDin), .ZeroE(ZeroE),
                          .PCSrcE(PCSrcE), .ImmSrcD(ImmSrcD), .ALUSrcE(ALUSrcE), .ALUControlE(ALUControlE),
                          .MemWriteM(MemWriteM), .ResultSrcW(ResultSrcW), .RegWriteW(RegWriteW), .RegWriteM(RegWriteM),
                          .ResultSrcE0(ResultSrcE0));
  
  datapath dp(.clk(clk), .reset(reset), .RegWriteW(RegWriteW), .InstrD(InstrD),
              .RegWriteM(RegWriteM), .MemWriteM(MemWriteM), .ALUSrcE(ALUSrcE), .PCSrcE(PCSrcE),
              .ResultSrcW(ResultSrcW), .ImmSrcD(ImmSrcD), .ALUControlE(ALUControlE), .ZeroE(ZeroE), .PCF(PCF), 
              .WriteDataM(WriteDataM), .ALUResultM(ALUResultM), .Instr(Instr), .ReadData(ReadData), .ForwardAE(ForwardAE),
              .ForwardBE(ForwardBE), .StallF(StallF), .StallD(StallD), .FlushE(FlushE), .RSE0in(ResultSrcE0),
              .Rs1E(Rs1E), .Rs2E(Rs2E), .RdM(RdM), .RdE(RdE), .RdW(RdW), .Rs1D(Rs1D), .Rs2D(Rs2D),
              .RegWriteMout(RegWriteMout), .RegWriteWout(RegWriteWout), .RSE0out(ResultSrcE0out), .PCSrcEout(PCSrcEout));
  
  hazard_unit h(.Rs1D(Rs1D), .Rs2D(Rs2D), .Rs1E(Rs1E), .Rs2E(Rs2E), .RdE(RdE), .RdM(RdM), .RdW(RdW), .RegWriteM(RegWriteMout),
                .RegWriteW(RegWriteWout), .PCSrcE(PCSrcEout), .ResultSrcE0(ResultSrcE0out), .StallF(StallF), .StallD(StallD),
                .FlushD(FlushD), .FlushE(FlushE), .ForwardAE(ForwardAE), .ForwardBE(ForwardBE));  
  
  // DataAdr is connected to ALUResult
  assign DataAdr = ALUResultM;
  assign WriteData = WriteDataM;
  assign MemWrite = MemWriteM;
  assign PC = PCF;
  
  
  integer cycle;
initial cycle = 0;

always @(posedge clk) begin
  cycle = cycle + 1;
  $display("Cycle=%0d x2=%h x3=%h x4=%h x5=%h x7=%h x9=%h", cycle,
           a.rvsingle.dp.rf.rf[2], a.rvsingle.dp.rf.rf[3], a.rvsingle.dp.rf.rf[4], a.rvsingle.dp.rf.rf[5],
           a.rvsingle.dp.rf.rf[7], a.rvsingle.dp.rf.rf[9]);

  // FETCH
  $display("  F: PCF=%h InstrF=%h", PCF, Instr);

  // DECODE
  $display("  D: PCD=%h InstrD=%h Rs1D=%0d Rs2D=%0d RdD=%0d BranchD=%0d ImmSrcD=%b ImmExtD=%h Opcode=%b RegWriteD=%b",
           dp.PCD, dp.InstrD, dp.Rs1D, dp.Rs2D, dp.RdD, cr.BranchD, dp.ImmSrcD, dp.ImmExtD, dp.InstrD[6:0], cr.RegWriteD);

  // EXECUTE
  $display("  E: PCE=%h RdE=%0d Rs1E=%0d RS2E=%0d SrcAE=%h SrcBE=%h ALUResultE=%h ZeroE=%b BranchE=%0d ImmExtE=%h RegWriteE=%b",
           dp.PCE, dp.RdE, dp.Rs1E, dp.Rs2E, dp.SrcAE, dp.SrcBE, dp.ALUResultE, dp.ZeroE,  cr.BranchE, dp.ImmExtE, cr.RegWriteE);

  // MEMORY
  $display("  M: RdM=%0d ALUResultM=%h WriteDataM=%h, RegWriteM=%b",
           dp.RdM, dp.ALUResultM, dp.WriteDataM, cr.RegWriteM);

  // WRITEBACK
  $display("  W: RdW=%0d ResultW=%h RegWriteW=%b",
           dp.RdW, dp.ResultW, cr.RegWriteW);

  //HAZARD
  $display("  HAZARD: StallF=%b StallD=%b FlushD=%b FlushE=%b ForwardAE=%b ForwardBE=%b",
           h.StallF, h.StallD, h.FlushD, h.FlushE, h.ForwardAE, h.ForwardBE);

  $display("-----------------------------------------------------");
end
endmodule

module top(input clk, reset, output [31:0] WriteData, DataAdr, output MemWrite);
  wire [31:0] PC, Instr, ReadData; 
  // instantiate processor and memories
  riscvsingle rvsingle(.clk(clk), .reset(reset), .PC(PC), .Instr(Instr), .MemWrite(MemWrite), 
    .DataAdr(DataAdr), .WriteData(WriteData), .ReadData(ReadData)); 
  imem imem(.a(PC), .rd(Instr));   
  dmem dmem(.clk(clk), .we(MemWrite), .a(DataAdr), .wd(WriteData), .rd(ReadData));
  
endmodule