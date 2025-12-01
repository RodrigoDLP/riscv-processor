// Code your design here
module alu32(a, b, ALUControl, f, flags);
  input [31:0] a, b; input [2:0] ALUControl; output [31:0] f; output [4:0] flags;
  wire [31:0] f1, f2, f3, f4; wire inexact1, inexact2, inexact3, inexact4, inexact, pureoverflow, pureunderflow;
  addition32template m1(.a(a), .b(b), .f(f1), .inexact(inexact1));
  subtraction32template m2(.a(a), .b(b), .f(f2), .inexact(inexact2));
  multiplication m3(.a(a), .b(b), .f(f3), .inexact(inexact3));
  division m4(.a(a), .b(b), .f(f4), .inexact(inexact4));
  mux41 #(.N(32)) mu(.a(f1), .b(f2), .c(f3), .d(f4), .s0(ALUControl[1]), .s1(ALUControl[0]), .f(f));
  mux41 #(.N(1)) mu_inexact(.a(inexact1), .b(inexact2), .c(inexact3), .d(inexact4), .s0(ALUControl[1]), .s1(ALUControl[0]), .f(inexact));
  //overflow, underflow, nan, div0, inexact
  assign pureoverflow = !((a[30:23] == 255 && a[22:0] == 0)||(b[30:23] == 255 && b[22:0] == 0)) && f[30:23] == 255 && f[22:0] == 0;
  assign pureunderflow = (ALUControl == 3'b001) && (a != b) && (f[30:0] == 0);
  assign flags[4] = pureoverflow;
  assign flags[3] = pureunderflow || ((f[30:23] == 0) && inexact);
  assign flags[2] = (f[30:23] == 255) && (f[22:0] != 0);
  assign flags[1] = (ALUControl == 3'b011) && (b[30:0] == 0);
  assign flags[0] = inexact | pureoverflow | pureunderflow;
endmodule

module addition32template(a, b, f, inexact);
  input [31:0] a, b; output [31:0] f; output inexact;
  wire [31:0] w1, w2; wire inexact1, inexact2;
  addition insideadd1(.a(a), .b(b), .f(w1), .inexact(inexact1));
  subtraction insidesubs1(.a(a), .b(b), .f(w2), .inexact(inexact2));
  assign f = a[31] ^ b[31] ? w2 : w1;
  assign inexact = a[31] ^ b[31] ? inexact2 : inexact1;
endmodule

module subtraction32template(a, b, f, inexact);
  input [31:0] a, b; output [31:0] f; output inexact;
  wire [31:0] w1, w2; wire inexact1, inexact2;
  addition insideadd1(.a(a), .b({~b[31], b[30:0]}), .f(w1), .inexact(inexact1));
  subtraction insidesubs1(.a(a), .b({~b[31], b[30:0]}), .f(w2), .inexact(inexact2));
  assign f = a[31] ^ ~b[31] ? w2 : w1;
  assign inexact = a[31] ^ ~b[31] ? inexact2 : inexact1;
endmodule

module mux21 #(parameter N=1)(a, b, sel, f);
  input [N-1:0] a, b; input sel; output [N-1:0] f;
  assign f = sel ? b : a; //hecho así para que no haya problemas con a y b arrays
endmodule
   
module mux41 #(parameter N=1)(a, b, c, d, s0, s1, f);
  input [N-1:0] a, b, c, d; input s0, s1; output [N-1:0] f; wire [N-1:0] w1, w2;
  mux21 #(.N(N)) mu1(.a(a), .b(b), .sel(s1), .f(w1));
  mux21 #(.N(N)) mu2(.a(c), .b(d), .sel(s1), .f(w2));
  mux21 #(.N(N)) mu3(.a(w1), .b(w2), .sel(s0), .f(f));
endmodule


module rounder32(min, expin, guardbit, roundbit, stickybit, mout, expout);
  input [22:0] min; input [7:0] expin; input guardbit, roundbit, stickybit;
  output [22:0] mout; output [7:0] expout;
  wire roundup = guardbit && (roundbit || stickybit || min[0]);
  wire [23:0] mantround;
  add #(.N(23)) addmantroundup(.a(min), .b({22'b0, roundup}), .f({mantround}));
  wire carry = mantround[23];
  wire [22:0] mantfinal = carry ? mantround[23:1] : mantround[22:0];
  wire [7:0] expplusone;
  addone #(.N(8)) expaddone(.a(expin), .f(expplusone));
  wire [7:0] exptmp; assign exptmp = carry ? expplusone : expin;
  assign mout = (expin != 255 && exptmp == 255) ? 0 : mantfinal;
  assign expout = (expin != 255 && exptmp == 255) ? 255 : exptmp;
endmodule

module lzd64(a, b);
  input [63:0] a; output[5:0] b;
  wire [31:0] w32; wire [15:0] w16; wire [7:0] w8;
  wire [3:0] w4; wire [1:0] w2;
  assign w32 = (a[63:32] == 0) ? a[31:0] : a[63:32];
  assign w16 = (w32[31:16] == 0) ? w32[15:0] : w32[31:16];
  assign w8 = (w16[15:8] == 0) ? w16[7:0] : w16[15:8];
  assign w4 = (w8[7:4] == 0) ? w8[3:0] : w8[7:4];
  assign w2 = (w4[3:2] == 0) ? w4[1:0] : w4[3:2];
  assign b[5] = (a[63:32] == 0);
  assign b[4] = (w32[31:16] == 0);
  assign b[3] = (w16[15:8] == 0);
  assign b[2] = (w8[7:4] == 0);
  assign b[1] = (w4[3:2] == 0);
  assign b[0] = (w2[1] == 0);
endmodule

module lzd32(a, b);
  input [31:0] a; output[4:0] b;
  wire [15:0] w16; wire [7:0] w8;
  wire [3:0] w4; wire [1:0] w2;
  assign w16 = (a[31:16] == 0) ? a[15:0] : a[31:16];
  assign w8 = (w16[15:8] == 0) ? w16[7:0] : w16[15:8];
  assign w4 = (w8[7:4] == 0) ? w8[3:0] : w8[7:4];
  assign w2 = (w4[3:2] == 0) ? w4[1:0] : w4[3:2];
  assign b[4] = (a[31:16] == 0);
  assign b[3] = (w16[15:8] == 0);
  assign b[2] = (w8[7:4] == 0);
  assign b[1] = (w4[3:2] == 0);
  assign b[0] = (w2[1] == 0);
endmodule

module shiftrightsticky #(parameter N=48, SHIFTBITS=9)(a, b, f, sticky);
  input [N-1:0] a; input[SHIFTBITS-1:0] b; output [N-1:0] f; output sticky;
  wire [(N*2-1):0] beforeshift, aftershift;
  assign beforeshift[(N*2-1):N] = a;
  assign beforeshift[N-1:0] = 0;
  assign aftershift = beforeshift >> b;
  assign f = aftershift[(N*2-1):N];
  assign sticky = |aftershift[N-1:0];
endmodule

module lt8(a, b, f);
  input [7:0] a, b; output f;
  assign f = a < b;
endmodule

module lessthan #(parameter N=8)(a, b, f);
  input [N-1:0] a, b; output f;
  assign f = a < b;
endmodule


module subst8abs(a, b, f);
  input [7:0] a, b; output [7:0] f;
  assign f = a-b;
endmodule

module mnorm(a, b, ma, mb, expout, stickyout);
  input [31:0] a, b; output [46:0] ma, mb; output [7:0] expout; output stickyout;
  wire l;
  wire [46:0] ma1, mb1;
  wire [7:0] expdiff, expdiff1, expdiff2;
  assign ma1[46] = (a[30:23] != 0); assign mb1[46] = (b[30:23] != 0);
  assign ma1[22:0] = 0; assign mb1[22:0] = 0;
  assign ma1[45:23] = a[22:0]; assign mb1[45:23] = b[22:0];
  lessthan #(.N(8)) lt(.a(a[30:23]), .b(b[30:23]), .f(l));
  subtract #(.N(8)) subst(.a(a[30:23]), .b(b[30:23]), .f(expdiff1));
  subtract #(.N(8)) subst2(.a(b[30:23]), .b(a[30:23]), .f(expdiff2));
  assign expdiff = l ? expdiff2 : expdiff1;
  assign expout = l ? b[30:23] : a[30:23];
  wire stickyma, stickymb;
  wire [46:0] shiftedma, shiftedmb;
  shiftrightsticky #(.N(47), .SHIFTBITS(8)) srsma(.a(ma1), .b(expdiff), .f(shiftedma), .sticky(stickyma));
  shiftrightsticky #(.N(47), .SHIFTBITS(8)) srsmb(.a(mb1), .b(expdiff), .f(shiftedmb), .sticky(stickymb));
  assign ma = l ? shiftedma : ma1;
  assign mb = l ? mb1 : shiftedmb;
  assign stickyout = l ? stickyma : stickymb;
  //assign ma = l ? ma1 >> expdiff : ma1;
  //assign mb = l ? mb1 : mb1 >> expdiff;
endmodule

module add #(parameter N=48) (a, b, f);
  input [N-1:0] a, b; output [N:0] f;
  assign f = a+b;
endmodule

module subtract #(parameter N=8) (a, b, f);
  input [N-1:0] a, b; output [N-1:0] f;
  assign f = a-b;
endmodule

module addone #(parameter N=8) (a, f);
  input [N-1:0] a; output [N-1:0] f;
  assign f = a+1;
endmodule

module substone #(parameter N=8) (a, f);
  input [N-1:0] a; output [N-1:0] f;
  assign f = a-1;
endmodule

module tc #(parameter N=48)(a, f);
  input [N-1:0] a; output [N-1:0] f;
  wire [N-1:0] b; assign b = ~a;
  addone #(.N(N)) a1(.a(b), .f(f));
endmodule

module outnormadd(a, expin, stickyin, mout, expout, inexact);
  input [47:0] a; input [7:0] expin; input stickyin; output [22:0] mout; output [7:0] expout; output inexact;
  wire sticky2;
  wire [47:0] f;
  wire overflow, tonormal; wire [7:0] expplusone;
  assign overflow = (a[47] == 1'b1 && expin != 8'b11111111);
  assign tonormal = (expin == 0 && a[46] == 1'b1);
  addone #(.N(8)) a1(.a(expin), .f(expplusone));
  assign f = a >> overflow;
  assign sticky2 = (a[0] & overflow) | stickyin;
  wire [7:0] expnorm;
  assign expnorm = (overflow || tonormal) ? expplusone : expin;
  rounder32 rnd(.min((expnorm==255) ? 23'b0 : f[45:23]), .expin(expnorm), .guardbit((expnorm==255) ? 1'b0 : f[22]), .roundbit((expnorm==255) ? 1'b0 : f[21]), .stickybit((expnorm==255) ? 1'b0 : ((|f[20:0]) | sticky2)), .mout(mout), .expout(expout));
  assign inexact = sticky2 | (|f[22:0]);
endmodule

module outnormsubst(a, expin, stickyin, mout, expout, inexact);
  input [47:0] a; input [7:0] expin; input stickyin; output [22:0] mout; output [7:0] expout; output inexact;
  wire [5:0] shiftby;
  wire shifttoobig;
  wire [7:0] outsubsexp;
  wire [47:0] f;
  lzd64 lz(.a({a[46:0], 17'b0}), .b(shiftby)); //el primer bit es de los signos, por lo que se cuenta a partir del segundo
  lessthan #(.N(8)) lt(.a(expin), .b({2'b0, shiftby}), .f(shifttoobig));
  subtract #(.N(8)) substexp(.a(expin), .b({2'b0, shiftby}), .f(outsubsexp));
  wire [7:0] expnorm;
  assign expnorm = shifttoobig ? 8'b00000000 : outsubsexp;
  assign f = shifttoobig ? a << expin : a << shiftby;
  rounder32 rnd(.min(f[45:23]), .expin(expnorm), .guardbit(f[22]), .roundbit(f[21]), .stickybit(|f[20:0]),
                 .mout(mout), .expout(expout));
  assign inexact = stickyin | (|f[22:0]);
endmodule

module addition(a, b, f, inexact);
  input [31:0] a, b; output [31:0] f; output inexact;
  wire [46:0] ma1, mb1; wire [7:0] expout1;
  wire [47:0] sum1;
  wire [22:0] fmprev;
  wire ainf, binf, anan, bnan;
  assign ainf = a[30:23] == 255 && a[22:0] == 0;
  assign binf = b[30:23] == 255 && b[22:0] == 0;
  assign anan = a[30:23] == 255 && a[22:0] != 0;
  assign bnan = b[30:23] == 255 && b[22:0] != 0;
  wire stickynorm1;
  mnorm norm1(.a(a), .b(b), .ma(ma1), .mb(mb1), .expout(expout1), .stickyout(stickynorm1));
  add #(.N(47)) add1(.a(ma1), .b(mb1), .f(sum1));
  outnormadd norm2(.a(sum1), .expin(expout1), .stickyin(stickynorm1), .mout(fmprev[22:0]), .expout(f[30:23]), .inexact(inexact));
  assign f[31] = a[31];
  assign f[22:0] = (anan || bnan) ? {1'b1, 22'b0} : ((ainf || binf) ? 0 : fmprev[22:0]);
endmodule

module subtraction(a, b, f, inexact);
  input [31:0] a, b; output [31:0] f; output inexact;
  wire [31:0] tca1, tcb1, a1, b1;
  wire [46:0] ma1, mb1;
  wire [7:0] expout1;
  wire [47:0] sub1, tcsub1, sub2, smanta1, smantb1, tcma, tcmb;
  wire dumpsign, dumpcout, ainf, binf, anan, bnan, azero, bzero, equaloperands;
  wire [30:0] fprevunsigned;
  assign ainf = a[30:23] == 255 && a[22:0] == 0;
  assign binf = b[30:23] == 255 && b[22:0] == 0;
  assign anan = a[30:23] == 255 && a[22:0] != 0;
  assign bnan = b[30:23] == 255 && b[22:0] != 0;
  assign equaloperands = a[30:0] == b[30:0];
  wire stickynorm1;
  mnorm norm1(.a(a), .b(b), .ma(ma1), .mb(mb1), .expout(expout1), .stickyout(stickynorm1));
  tc #(.N(48)) tca(.a({1'b0, ma1}), .f(tcma));
  tc #(.N(48)) tcb(.a({1'b0, mb1}), .f(tcmb));
  assign smanta1 = a[31] == 1'b1 ? tcma : ma1;
  assign smantb1 = b[31] == 1'b1 ? tcmb : mb1;
  add #(.N(48)) add1(.a(smanta1), .b(smantb1), .f({dumpcout, sub1}));
  tc #(.N(48)) tcsub(.a(sub1), .f({dumpsign, tcsub1[46:0]}));
  assign tcsub1[47] = 1'b1;
  assign sub2 = (sub1[47] == 1'b1) ? tcsub1 : sub1;
  wire [47:0] sub2_debug;
    assign sub2_debug = sub2;
  outnormsubst norm2(.a(sub2_debug), .expin(expout1), .stickyin(stickynorm1), .mout(fprevunsigned[22:0]), .expout(fprevunsigned[30:23]), .inexact(inexact));
  assign f[31] = (fprevunsigned[30:0] == 0) ? 0 : sub2[47];
  assign f[30:23] = (ainf || binf || anan || bnan) ? 255 : (equaloperands ? 0 : fprevunsigned[30:23]);
  assign f[22:0] = ((anan || bnan || (ainf && binf)) ? {1'b1, 22'b0} : ((ainf || binf || equaloperands) ? 0 : fprevunsigned[22:0]));    
endmodule


 
module multiply(a, b, f);
  input [23:0] a, b; output [47:0] f; //pongo 47 y no 46 para que cuadre con la normalización de subtraction
  assign f = a * b;
endmodule

module divide(a, b, f);
  input [23:0] a, b; output [47:0] f;
  assign f = (b==0) ? 0 : ({a, 24'b0} / {24'b0, b}); //devuelve 0 como cualquier constante en caso sea div/0. no importa la normalización ni que esté desfasado a la izquierda, se hace después
endmodule

module division(a, b, f, inexact);
  input [31:0] a, b; output [31:0] f; output inexact;
  wire [9:0] tcexpsubbias, tcexpsubunbias;
  wire [9:0] expaunbias1, expbunbias1, expaunbias2, expbunbias2, expbunbiastc;
  wire [7:0] expfinal;
  wire [5:0] zerocount;
  wire [47:0] mquot, mquotnorm, mquotshiftright;
  wire expaunbiasdump, expbunbiasdump, expaunbias2dump, expbunbias2dump, expsubdump, tcexpsubbiasdump, expaugmentdump;
  wire lt, gt, bgreater;
  wire ainf, binf, anan, bnan, azero, bzero;
  assign ainf = a[30:23] == 255 && a[22:0] == 0;
  assign binf = b[30:23] == 255 && b[22:0] == 0;
  assign anan = a[30:23] == 255 && a[22:0] != 0;
  assign bnan = b[30:23] == 255 && b[22:0] != 0;
  assign azero = a[30:0] == 0;
  assign bzero = b[30:0] == 0;
  wire [23:0] shiftedma; wire [23:0] shiftedmb;
  //mantisas
  
  wire [4:0] zerocountma, zerocountmb;
  wire [9:0] zerocountmatc, zerocountmbtc;
  lzd32 lzma(.a({(a[30:23] != 0), a[22:0], 8'b0}), .b(zerocountma)); //solo aplica para subnormales por lo que el 0 al inicio es correcto, además para sumarle 1 al resultado
  lzd32 lzmb(.a({(b[30:23] != 0), b[22:0], 8'b0}), .b(zerocountmb));
  tc #(.N(10)) zerocountmatccalc(.a({5'b0, zerocountma}), .f(zerocountmatc));
  tc #(.N(10)) zerocountmbtccalc(.a({5'b0, zerocountmb}), .f(zerocountmbtc));
  assign shiftedma = ({(a[30:23] != 0),a[22:0]}) << zerocountma;
  assign shiftedmb = ({(b[30:23] != 0),b[22:0]}) << zerocountmb;
  divide mdiv(.a(shiftedma), .b(shiftedmb), .f(mquot));
 lessthan #(.N(24)) bgreatercalc(.a(shiftedma), .b(shiftedmb), .f(bgreater));
  add #(.N(10)) expaunbiascalc1(.a({2'b00, a[30:23]}), .b(10'b1110000001), .f({expaunbiasdump, expaunbias1}));
  add #(.N(10)) expbunbiascalc1(.a({2'b00, b[30:23]}), .b(10'b1110000001), .f({expbunbiasdump, expbunbias1}));
  add #(.N(10)) expaunbiascalc2(.a(expaunbias1), .b(zerocountmatc), .f({expaunbias2dump, expaunbias2}));
  add #(.N(10)) expbunbiascalc2(.a(expbunbias1), .b(zerocountmbtc), .f({expbunbias2dump, expbunbias2}));
   
 
  tc #(.N(10)) tcexpbunbiascalc(.a(expbunbias2), .f(expbunbiastc));
  add #(.N(10)) expsubcalc(.a(expaunbias2), .b(expbunbiastc), .f({expsubdump, tcexpsubunbias}));
  wire [9:0] finalbias; assign finalbias = bgreater ? 126 : 127;
  add #(.N(10)) expsubrebiascalc(.a(tcexpsubunbias), .b(finalbias), .f({tcexpsubbiasdump, tcexpsubbias}));
  //comprobar que caiga en el rango 0-255. el primer bit no puede ser 1 y el segundo tampoco.
  assign lt = tcexpsubbias[9];
  assign gt = ~tcexpsubbias[9] & tcexpsubbias[8];
  wire [8:0] expreduce, expaugment;
  tc #(.N(10)) expsubcomplementcalc(.a(tcexpsubbias), .f({expaugmentdump, expaugment}));
  assign expfinal = lt ? 0 : (gt ? 255 : tcexpsubbias[7:0]);
  wire [5:0] zerocountquot;
  lzd64 lzdquot(.a({mquot, 16'b0}), .b(zerocountquot));
  assign mquotnorm = mquot << zerocountquot;
 
   wire mquotrightsticky;
  shiftrightsticky #(.N(48), .SHIFTBITS(9)) srs1(.a(mquotnorm), .b(expaugment), .f(mquotshiftright), .sticky(mquotrightsticky));
  wire [47:0] mextfinal; wire [7:0] expfinalrounded; wire [22:0] mfinalrounded;
  assign mextfinal = lt ? {mquotshiftright[46:0], mquotrightsticky} : ((tcexpsubbias == 0) ? mquotnorm : {mquotnorm[46:0], 1'b0}); //mquotrightsticky puesto al final en vez de 0 para ahorrar wires
  rounder32 rnd(.min(mextfinal[47:25]), .expin(expfinal), .guardbit(mextfinal[24]), .roundbit(mextfinal[23]), .stickybit(|mextfinal[22:0]), .mout(mfinalrounded), .expout(expfinalrounded));
  assign inexact = (|mextfinal[24:0]);
  assign f[31] = a[31] ^ b[31];
  assign f[30:23] = (a[30:23] == 255 || bzero || bnan) ? 255 :
    ((azero || binf) ? 0: expfinalrounded);
  assign f[22:0] = (anan || bnan || (azero && bzero) || (ainf && binf)) ? {1'b1, 22'b0} : ((ainf || binf || gt || azero || bzero || (expfinalrounded==255)) ? 0 : mfinalrounded);
endmodule

module multiplication(a, b, f, inexact);
  input [31:0] a, b; output [31:0] f; output inexact;
  wire [8:0] expsum; wire [7:0] expfinal;
  wire lt, gt;
  wire [47:0] mprod, mprodshiftleft, mprodshiftright;
  wire [5:0] zerocount;
  wire [7:0] expsumoffset;
  wire [9:0] expsum1; wire [8:0] expaugment;
  wire highexpaugment, ainf, binf, anan, bnan, azero, bzero;
  assign ainf = a[30:23] == 255 && a[22:0] == 0;
  assign binf = b[30:23] == 255 && b[22:0] == 0;
  assign anan = a[30:23] == 255 && a[22:0] != 0;
  assign bnan = b[30:23] == 255 && b[22:0] != 0;
  assign azero = a[30:0] == 0;
  assign bzero = b[30:0] == 0;
  multiply mmultip(.a({(a[30:23] != 0),a[22:0]}), .b({(b[30:23] != 0), b[22:0]}), .f(mprod));
  lzd64 lz(.a({mprod, 16'b0}), .b(zerocount));
  add #(.N(8)) expadder(.a(a[30:23]), .b(b[30:23]), .f(expsum));
  subtract #(.N(8)) expoffsetcalc(.a(8'b10000010), .b({2'b0, zerocount}), .f(expsumoffset));
  add #(.N(9)) expoffsetadd(.a(expsum), .b({1'b0, expsumoffset}), .f(expsum1));
  lessthan #(.N(10)) toosmallcalc(.a(expsum1), .b(10'b0100000000), .f(lt)); //reducible a que los dos primeros bits sean cero
  lessthan #(.N(10)) toobigcalc(.a(10'b1000000000), .b(expsum1), .f(gt)); //reducible a una única comparación de 1 bit
  //si se pasa del rango de 0-255 (256-511), se trunca a 0 o 255 y se desplaza la mantisa
  subtract #(.N(9)) expaugmentcalc(.a(9'b100000000), .b(expsum1[8:0]), .f(expaugment));
  //se puede resumir a simplemente los 9 bits a la derecha de expsum1
  assign expfinal = lt ? 0 : (gt ? 255 : expsum1[7:0]); //no se necesita quitarle 256 ya que está en el rango 256:511, simplemente se obtienen los últimos 8 bits
 
    //si expaugment > zerocount y lt == 1, shift right expaugment - zerocount
    //si expaugment < zerocount y lt == 1, shift left zerocount - expaugment
    //si lt == 0, shift left zerocount
  wire [8:0] rsamount, lsamountlt; wire [8:0] lsamount;
  subtract #(.N(9)) rscalc(.a(expaugment), .b({3'b0, zerocount}), .f(rsamount));
  subtract #(.N(9)) lsltcalc(.a({3'b0, zerocount}), .b(expaugment), .f(lsamountlt));
  //nunca hará nada cuando gt == 1, simplemente fija la mantisa a 0
  assign lsamount = lt ? lsamountlt : {3'b0, zerocount};
  assign mprodshiftleft = mprod << lsamount;
  wire mprodrightsticky;
  shiftrightsticky #(.N(48), .SHIFTBITS(9)) srs1(.a(mprod), .b(rsamount), .f(mprodshiftright), .sticky(mprodrightsticky));
  lessthan #(.N(9)) highexpaugmcalc(.a({3'b0, zerocount}), .b(expaugment), .f(highexpaugment));
 
  wire [47:0] mextfinal; wire [7:0] expfinalrounded; wire [22:0] mfinalrounded;
  assign mextfinal = (highexpaugment && lt) ? {mprodshiftright[46:0], mprodrightsticky} : ((expsum1 == 10'b0100000000) ? mprodshiftleft : {mprodshiftleft[46:0], 1'b0}); //mprodrightsticky puesto al final en vez de 0 para ahorrar wires
  rounder32 rnd(.min(mextfinal[47:25]), .expin(expfinal), .guardbit(mextfinal[24]), .roundbit(mextfinal[23]), .stickybit(|mextfinal[22:0]),
                .mout(mfinalrounded), .expout(expfinalrounded));
 
  assign inexact = (|mextfinal[24:0]);
  assign f[22:0] = (anan || bnan || (azero && binf) || (ainf && bzero)) ? {1'b1, 22'b0} : ((ainf || binf || azero || bzero || (expfinalrounded==255)) ? 0 : mfinalrounded);
  assign f[31] = a[31] ^ b[31];
  assign f[30:23] = (a[30:23] == 255 || b[30:23] == 255) ? 255 :
    ((azero || bzero) ? 0: expfinalrounded);
endmodule

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










module aludec(input opb5, input [2:0] funct3, input funct7b5, input [6:0] funct7, input [1:0] ALUOp, output [2:0] ALUControl);
  wire  RtypeSub; 
  reg [2:0] ALUControl_reg; 
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction
  assign ALUControl = ALUControl_reg;
  always @(*) case(ALUOp)
      2'b00: ALUControl_reg = 3'b000; // addition
      2'b01: ALUControl_reg = 3'b001; // subtraction
    2'b11: casex (funct7[6:2])
      5'b00000: ALUControl_reg = 3'b000;
      5'b00001: ALUControl_reg = 3'b001;
      5'b00010: ALUControl_reg = 3'b010;
      5'b00011: ALUControl_reg = 3'b011;
      5'b11010: ALUControl_reg = 3'b100;
      default: ALUControl_reg = 3'bxxx;
      	
    	endcase
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



module controller(input [6:0] op, input [2:0] funct3, input funct7b5, input [6:0] funct7,
                  output [1:0] ResultSrcD, output MemWriteD, JumpD, BranchD, ALUSrcD, RegWriteD,
                  output [2:0] ImmSrcD, output [2:0] ALUControlD, output ALUPickerD, FloatWriteDataD, FloatRegWriteD,
                 ForceR1ZeroD);
  wire [1:0] ALUOp; 
  wire Branch; 
  maindec md(.op(op), .ResultSrc(ResultSrcD), .MemWrite(MemWriteD), .Branch(BranchD),
             .ALUSrc(ALUSrcD), .RegWrite(RegWriteD), .Jump(JumpD), .ImmSrc(ImmSrcD), .ALUOp(ALUOp),
             .ALUPickerD(ALUPickerD), .FloatWriteDataD(FloatWriteDataD), .FloatRegWriteD(FloatRegWriteD),
             .ForceR1ZeroD(ForceR1ZeroD)); 
  aludec ad(.opb5(op[5]), .funct3(funct3), .funct7b5(funct7b5), .funct7(funct7), .ALUOp(ALUOp), .ALUControl(ALUControlD)); 
endmodule

module controller_registers(input clk, input reset,
                            input RegWriteD,
                            input [1:0] ResultSrcD,
                            input MemWriteD,
                            input JumpD,
                            input BranchD,
                            input [2:0] ALUControlD,
                            input ALUSrcD,
                            input [2:0] ImmSrc,
                            input ZeroE,
                            output PCSrcE, ResultSrcE0,
                            output [2:0] ImmSrcD,
                            output reg ALUSrcE,
                            output reg [2:0] ALUControlE,
                            output reg MemWriteM,
                            output reg [1:0] ResultSrcW,
                            output reg RegWriteW, RegWriteM,
                           input ALUPickerD, FloatWriteDataD, FloatRegWriteD,
                            output reg ALUPickerE, ALUPickerM, ALUPickerW, 
                            FloatWriteDataE, FloatRegWriteM, FloatRegWriteW,
                           input ForceR1ZeroD,
                           output reg ForceR1ZeroE);
  reg RegWriteE, MemWriteE, JumpE, BranchE, FloatRegWriteE;
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
    if (reset) ALUPickerE <= 0; else ALUPickerE <= ALUPickerD;
    if (reset) ALUPickerM <= 0; else ALUPickerM <= ALUPickerE;
    if (reset) ALUPickerW <= 0; else ALUPickerW <= ALUPickerM;
    if (reset) FloatWriteDataE <= 0; else FloatWriteDataE <= FloatWriteDataD;
    if (reset) FloatRegWriteE <= 0; else FloatRegWriteE <= FloatRegWriteD;
    if (reset) FloatRegWriteM <= 0; else FloatRegWriteM <= FloatRegWriteE;
    if (reset) FloatRegWriteW <= 0; else FloatRegWriteW <= FloatRegWriteM;
    if (reset) ForceR1ZeroE <= 0; else ForceR1ZeroE <= ForceR1ZeroD;
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
                input [1:0] ResultSrcW, 
                input [2:0] ALUControlE, ImmSrcD,
                output ZeroE,
                output reg [31:0] PCF, WriteDataM, InstrD,
                output reg [31:0] ALUResultM,
                input [31:0] Instr, ReadData,
                input [1:0] ForwardAE, ForwardBE,
               input StallF, StallD, FlushD, FlushE, RSE0in,
                output reg [4:0] Rs1E, Rs2E, RdM, RdE, RdW,
                output wire [4:0] Rs1D, Rs2D,
               output RegWriteMout, RegWriteWout, RSE0out, PCSrcEout,
               input ALUPickerE, ALUPickerM, ALUPickerW, FloatRegWriteM, FloatRegWriteW, FloatWriteDataE,
                input [1:0] FloatForwardAE, FloatForwardBE,
                output ALUPickerEout, ALUPickerMout, ALUPickerWout, FloatRegWriteMout, FloatRegWriteWout,
                input ForceR1ZeroE
               );
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
  wire [31:0] FRFRD1, FRFRD2;
  reg [31:0] FRD1E, FRD2E;
  regfile floatrf(.clk(clk), .reset(reset), .we3(FloatRegWriteW), .a1(InstrD[19:15]), .a2(InstrD[24:20]), .a3(RdW),
                  .wd3(ResultW), .rd1(FRFRD1), .rd2(FRFRD2));
  assign RdD = InstrD[11:7];
  assign Rs1D = InstrD[19:15];
  assign Rs2D = InstrD[24:20];
  extend ext(.instr(InstrD[31:7]), .immsrc(ImmSrcD), .immext(ImmExtD));
  //EXECUTE
  wire [31:0] correctedRD1E;
  mux2 #(WIDTH) luicorrectionmux(.d0(RD1E), .d1(32'b0), .s(ForceR1ZeroE), .y(correctedRD1E)); 
  mux3 #(WIDTH) forwardamux(.d0(correctedRD1E), .d1(ResultW), .d2(ALUResultM), .s(ForwardAE), .y(SrcAE));
  mux3 #(WIDTH) forwardbmux(.d0(RD2E), .d1(ResultW), .d2(ALUResultM), .s(ForwardBE), .y(WriteDataE));
  mux2 #(WIDTH) srcbmux(.d0(WriteDataE), .d1(ImmExtE), .s(ALUSrcE), .y(SrcBE));
  
  wire [31:0] FSrcAE, FSrcBE, FWriteDataE;
  mux3 #(WIDTH) floatforwardamux(.d0(FRD1E), .d1(ResultW), .d2(ALUResultM), .s(FloatForwardAE), .y(FSrcAE));
  mux3 #(WIDTH) floatforwardbmux(.d0(FRD2E), .d1(ResultW), .d2(ALUResultM), .s(FloatForwardBE), .y(FWriteDataE));
  mux2 #(WIDTH) floatsrcbmux(.d0(FWriteDataE), .d1(ImmExtE), .s(ALUSrcE), .y(FSrcBE));
  
  
  adder pcaddbranch(.a(PCE), .b(ImmExtE), .y(PCTargetE));
  alu alu(.a(SrcAE), .b(SrcBE), .alucontrol(ALUControlE), .result(ALUResultE), .zero(ZeroE));
  wire [31:0] floatResultE, FinalAluResult, FinalWriteDataE;
  wire [4:0] floatFlags;
  // alu32 floatalu(.a(FSrcAE), .b(FSrcBE), .ALUControl(ALUControlE), .f(floatResultE), .flags(floatFlags));

  // ALU: mALUma
  // =============
  mALUma falu(
    .clk(clk),
    .rst(reset),
    .op_A(FSrcAE),          
    .op_B(FSrcBE),      
    .op_A_int(SrcAE),
    .op_code(ALUControlE),        // Código operación: 000=ADD, 001=SUB, 010=MUL, 011=DIV
    .mode_fp(1'b1),              // 0=half(16-bit), 1=single(32-bit)
    .round_mode(1'b0),           // Modo redondeo: 0=nearest even
    .result(floatResultE),        
    .flags(floatFlags)
  );
  // =============
  mux2 #(WIDTH) alupicker(.d0(ALUResultE), .d1(floatResultE), .s(ALUPickerE), .y(FinalAluResult));
  mux2 #(WIDTH) writedatapicker(.d0(WriteDataE), .d1(FWriteDataE), .s(FloatWriteDataE), .y(FinalWriteDataE));
    //OUTPUT DEL WRITE DATA DE FLOAT. SE NECESITA COPIAR TODO DESDE RD1E Y RD2E Y LOS MUX3 PARA QUE AL FINAL RECIÉN SE ELIJA EL RESULTADO. LUEGO SE ELIGE ENTRE LOS WRITEDATA PARA TENER SOLO UN FINALWRITEDATAE. FALTA ACTUALIZAR RISCVSINGLE Y DAR EFECTO A FLOATREGWRITE
  //MEMORY & WRITEBACK
  mux3 #(WIDTH) resultmux(.d0(ALUResultW), .d1(ReadDataW), .d2(PCPlus4W), .s(ResultSrcW), .y(ResultW));
  
  assign RegWriteMout = RegWriteM;
  assign RegWriteWout = RegWriteW;
  assign RSE0out = RSE0in;
  assign PCSrcEout = PCSrcE;
  
  assign ALUPickerEout = ALUPickerE;
  assign ALUPickerMout = ALUPickerM;
  assign ALUPickerWout = ALUPickerW;
  assign FloatRegWriteMout = FloatRegWriteM;
  assign FloatRegWriteWout = FloatRegWriteW;
  
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
    if (FlushE || reset) FRD1E <= 0; else FRD1E <= FRFRD1;
    if (FlushE || reset) FRD2E <= 0; else FRD2E <= FRFRD2;
    if (FlushE || reset) RdE <= 0; else RdE <= RdD;
    if (FlushE || reset) ImmExtE <= 0; else ImmExtE <= ImmExtD;
    if (FlushE || reset) PCPlus4E <= 0; else PCPlus4E <= PCPlus4D;
    if (reset) ALUResultM <= 0; else ALUResultM <= FinalAluResult;
    if (reset) WriteDataM <= 0; else WriteDataM <= FinalWriteDataE;
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

module extend(input  [31:7] instr, input  [2:0]  immsrc, output [31:0] immext);
  reg [31:0] immext_reg; 
  assign immext = immext_reg;
  always @* case(immsrc) 
               // I-type 
      3'b000:   immext_reg = {{20{instr[31]}}, instr[31:20]}; 
               // S-type (stores)
      3'b001:   immext_reg = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      3'b010:   immext_reg = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      3'b011:   immext_reg = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
    3'b100: immext_reg = {instr[31:12], 12'b0};
      default: immext_reg = 32'bx; // undefined
    endcase             
endmodule


module hazard_unit(input [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
                   input RegWriteM, RegWriteW, PCSrcE, ResultSrcE0,
                   output StallF, StallD, FlushD, FlushE,
                   output [1:0] ForwardAE, ForwardBE, FloatForwardAE, FloatForwardBE,
                  input ALUPickerE, ALUPickerM, ALUPickerW, FloatRegWriteM, FloatRegWriteW);
  assign ForwardAE[1] = (Rs1E == RdM) & RegWriteM & (Rs1E != 0);
  assign ForwardAE[0] = (Rs1E == RdW) & RegWriteW & (Rs1E != 0) & !((Rs1E == RdM) & RegWriteM);
  assign ForwardBE[1] = (Rs2E == RdM) & RegWriteM & (Rs2E != 0);
  assign ForwardBE[0] = (Rs2E == RdW) & RegWriteW & (Rs2E != 0) & !((Rs2E == RdM) & RegWriteM);
  assign FloatForwardAE[1] = (Rs1E == RdM) & FloatRegWriteM & ALUPickerM & (Rs1E != 0);
  assign FloatForwardAE[0] = (Rs1E == RdW) & FloatRegWriteW & ALUPickerE & (Rs1E != 0) 
    & !((Rs1E == RdM) & FloatRegWriteM & ALUPickerM);
  assign FloatForwardBE[1] = (Rs2E == RdM) & FloatRegWriteM & ALUPickerM & (Rs2E != 0);
  assign FloatForwardBE[0] = (Rs2E == RdW) & FloatRegWriteW & ALUPickerE & (Rs2E != 0) 
    & !((Rs2E == RdM) & FloatRegWriteM & ALUPickerM);
  wire lwStall;
  assign lwStall = ResultSrcE0 & ((Rs1D == RdE) | (Rs2D == RdE));
  assign StallF = lwStall;
  assign StallD = lwStall;
  assign FlushD = PCSrcE;
  assign FlushE = lwStall | PCSrcE;
endmodule


module imem(input  [31:0] a, output [31:0] rd);
  reg [31:0] RAM[63:0];
  integer j;
  initial begin
  
  for (j=0; j<64; j=j+1) RAM[j] = 32'b0;
    $readmemh("instructions.hex",RAM); 
  end
  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module maindec(input  [6:0] op, output [1:0] ResultSrc, output MemWrite, output Branch, ALUSrc,
               output RegWrite, Jump, output [2:0] ImmSrc, output [1:0] ALUOp, output ALUPickerD, FloatWriteDataD, FloatRegWriteD,
              ForceR1ZeroD); 
  reg [15:0] controls; 
  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump, ALUPickerD, FloatWriteDataD, FloatRegWriteD, ForceR1ZeroD} = controls; 
  always @* case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump_ALUPickerD_FloatWriteDataD_FloatRegWriteD_ForceR1ZeroD
      7'b0000011: controls <= 16'b1_000_1_0_01_0_00_0_0_0_0_0; // lw
      7'b0100011: controls <= 16'b0_001_1_1_00_0_00_0_0_0_0_0; // sw
      7'b0110011: controls <= 16'b1_xxx_0_0_00_0_10_0_0_0_0_0; // R-type
      7'b1100011: controls <= 16'b0_010_0_0_00_1_01_0_0_0_0_0; // beq
      7'b0010011: controls <= 16'b1_000_1_0_00_0_10_0_0_0_0_0; // I-type ALU
      7'b1101111: controls <= 16'b1_011_0_0_10_0_00_1_0_0_0_0; // jal
    7'b0110111: controls <= 16'b1_100_1_0_00_0_00_0_0_0_0_1; // lui
    7'b1010011: controls <= 16'b1_xxx_0_0_00_0_11_0_1_0_1_0; // R-type FLOAT
    7'b0000111: controls <= 16'b1_000_1_0_01_0_00_0_0_0_1_0; // lw FLOAT
    7'b0100111: controls <= 16'b0_001_1_1_00_0_00_0_0_1_0_0; // sw FLOAT
    7'b0000000: controls <= 16'b0; //safety measure for after reset
      default:    controls <= 16'bx_xxx_x_x_xx_x_xx_x_x_x_x_x; // non-implemented instruction
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
  wire [1:0] ResultSrcD;
  wire [2:0] ImmSrcDin, ImmSrcD; 
  wire [1:0] ResultSrcW;
  wire [2:0] ALUControlD;
  wire [2:0] ALUControlE;
  wire PCSrcE; 
  wire [31:0] InstrD, PCF, WriteDataM, ALUResultM;
  wire [1:0] ForwardAE, ForwardBE;
  wire StallF, StallD, FlushD, FlushE, RegWriteMout, RegWriteWout, ResultSrcE0out, PCSrcEout;
  wire [4:0] Rs1E, Rs2E, RdM, RdE, RdW, Rs1D, Rs2D; 
   
  
  wire ALUPickerD, ALUPickerE, ALUPickerM, ALUPickerW, ALUPickerEout, ALUPickerMout, ALUPickerWout;
  wire FloatRegWriteD, FloatWriteDataD, FloatRegWriteM, FloatRegWriteW, FloatRegWriteMout, FloatRegWriteWout, FloatWriteDataE;
  wire [1:0] FloatForwardAE, FloatForwardBE;
  wire ForceR1ZeroD, ForceR1ZeroE;
  
  controller c(.op(InstrD[6:0]), .funct3(InstrD[14:12]), .funct7b5(InstrD[30]), .funct7(InstrD[31:25]),
               .ResultSrcD(ResultSrcD), .MemWriteD(MemWriteD),
               .JumpD(JumpD), .BranchD(BranchD), .ALUSrcD(ALUSrcD), .RegWriteD(RegWriteD), .ImmSrcD(ImmSrcDin),
               .ALUControlD(ALUControlD), .ALUPickerD(ALUPickerD), .FloatWriteDataD(FloatWriteDataD), .FloatRegWriteD(FloatRegWriteD), .ForceR1ZeroD(ForceR1ZeroD));
  
  
  /*
  output ALUPickerD, FloatWriteDataD, FloatRegWriteD
  */
  controller_registers cr(.clk(clk), .reset(reset), .RegWriteD(RegWriteD), .ResultSrcD(ResultSrcD), .MemWriteD(MemWriteD),
                          .JumpD(JumpD),
                          .BranchD(BranchD), .ALUControlD(ALUControlD), .ALUSrcD(ALUSrcD), .ImmSrc(ImmSrcDin), .ZeroE(ZeroE),
                          .PCSrcE(PCSrcE), .ImmSrcD(ImmSrcD), .ALUSrcE(ALUSrcE), .ALUControlE(ALUControlE),
                          .MemWriteM(MemWriteM), .ResultSrcW(ResultSrcW), .RegWriteW(RegWriteW), .RegWriteM(RegWriteM),
                          .ResultSrcE0(ResultSrcE0), .ALUPickerD(ALUPickerD), .FloatWriteDataD(FloatWriteDataD),
                          .FloatRegWriteD(FloatRegWriteD), .ALUPickerE(ALUPickerE), .ALUPickerM(ALUPickerM),
                          .ALUPickerW(ALUPickerW), .FloatWriteDataE(FloatWriteDataE), .FloatRegWriteM(FloatRegWriteM),
                          .FloatRegWriteW(FloatRegWriteW), .ForceR1ZeroD(ForceR1ZeroD), .ForceR1ZeroE(ForceR1ZeroE));
  
 
  
  
  datapath dp(.clk(clk), .reset(reset), .RegWriteW(RegWriteW), .InstrD(InstrD),
              .RegWriteM(RegWriteM), .MemWriteM(MemWriteM), .ALUSrcE(ALUSrcE), .PCSrcE(PCSrcE),
              .ResultSrcW(ResultSrcW), .ImmSrcD(ImmSrcD), .ALUControlE(ALUControlE), .ZeroE(ZeroE), .PCF(PCF), 
              .WriteDataM(WriteDataM), .ALUResultM(ALUResultM), .Instr(Instr), .ReadData(ReadData), .ForwardAE(ForwardAE),
              .ForwardBE(ForwardBE), .StallF(StallF), .StallD(StallD), .FlushD(FlushD), .FlushE(FlushE), .RSE0in(ResultSrcE0),
              .Rs1E(Rs1E), .Rs2E(Rs2E), .RdM(RdM), .RdE(RdE), .RdW(RdW), .Rs1D(Rs1D), .Rs2D(Rs2D),
              .RegWriteMout(RegWriteMout), .RegWriteWout(RegWriteWout), .RSE0out(ResultSrcE0out), .PCSrcEout(PCSrcEout), .ALUPickerE(ALUPickerE), .ALUPickerM(ALUPickerM), .ALUPickerW(ALUPickerW), .FloatRegWriteM(FloatRegWriteM),
              .FloatRegWriteW(FloatRegWriteW), .FloatWriteDataE(FloatWriteDataE), .FloatForwardAE(FloatForwardAE),
              .FloatForwardBE(FloatForwardBE), .ALUPickerEout(ALUPickerEout), .ALUPickerMout(ALUPickerMout),
              .ALUPickerWout(ALUPickerWout), .FloatRegWriteMout(FloatRegWriteMout), .FloatRegWriteWout(FloatRegWriteWout),
              .ForceR1ZeroE(ForceR1ZeroE));
  

  hazard_unit h(.Rs1D(Rs1D), .Rs2D(Rs2D), .Rs1E(Rs1E), .Rs2E(Rs2E), .RdE(RdE), .RdM(RdM), .RdW(RdW), .RegWriteM(RegWriteMout),
                .RegWriteW(RegWriteWout), .PCSrcE(PCSrcEout), .ResultSrcE0(ResultSrcE0out), .StallF(StallF), .StallD(StallD),
                .FlushD(FlushD), .FlushE(FlushE), .ForwardAE(ForwardAE), .ForwardBE(ForwardBE), .FloatForwardAE(FloatForwardAE),
                .FloatForwardBE(FloatForwardBE), .ALUPickerE(ALUPickerEout), .ALUPickerM(ALUPickerMout), .ALUPickerW(ALUPickerWout),
                .FloatRegWriteM(FloatRegWriteMout), .FloatRegWriteW(FloatRegWriteWout));  
  
  // DataAdr is connected to ALUResult
  assign DataAdr = ALUResultM;
  assign WriteData = WriteDataM;
  assign MemWrite = MemWriteM;
  assign PC = PCF;
  
  
  integer cycle;
initial cycle = 0;

always @(posedge clk) begin
  cycle = cycle + 1;
  $display("Cycle=%0d x2=%h x3=%h x4=%h x5=%h x7=%h x9=%h floatx8=%h floatx9=%h", cycle,
           a.rvsingle.dp.rf.rf[2], a.rvsingle.dp.rf.rf[3], a.rvsingle.dp.rf.rf[4], a.rvsingle.dp.rf.rf[5],
           a.rvsingle.dp.rf.rf[7], a.rvsingle.dp.rf.rf[9], a.rvsingle.dp.floatrf.rf[8], a.rvsingle.dp.floatrf.rf[9]);

  // FETCH
  $display("  F: PCF=%h InstrF=%h", PCF, Instr);

  // DECODE
  $display("  D: PCD=%h InstrD=%h Rs1D=%0d Rs2D=%0d RdD=%0d BranchD=%0d ImmSrcD=%b ImmExtD=%h Opcode=%b RegWriteD=%b FRFRD1=%h FRFRD2=%h ALUPickerD=%b FloatRegWriteD=%b",
           dp.PCD, dp.InstrD, dp.Rs1D, dp.Rs2D, dp.RdD, cr.BranchD, dp.ImmSrcD, dp.ImmExtD, dp.InstrD[6:0], cr.RegWriteD, dp.FRFRD1, dp.FRFRD2, cr.ALUPickerD, cr.FloatRegWriteD);

  // EXECUTE
  $display("  E: PCE=%h RdE=%0d Rs1E=%0d RS2E=%0d SrcAE=%h SrcBE=%h ALUResultE=%h ZeroE=%b BranchE=%0d ImmExtE=%h RegWriteE=%b FinalAluResult=%h FinalWriteDataE=%h FSrcAE=%h FSrcBE=%h ALUControlE=%b floatResultE=%h FRD1E=%h FRD2E=%h ALUPickerE=%b FloatRegWriteE=%b WriteDataE=%h",
           dp.PCE, dp.RdE, dp.Rs1E, dp.Rs2E, dp.SrcAE, dp.SrcBE, dp.ALUResultE, dp.ZeroE,  cr.BranchE, dp.ImmExtE, cr.RegWriteE, dp.FinalAluResult, dp.FinalWriteDataE, dp.FSrcAE, dp.FSrcBE, dp.ALUControlE, dp.floatResultE, dp.FRD1E, dp.FRD2E, cr.ALUPickerE, cr.FloatRegWriteE, dp.WriteDataE);

  
  //alu32 floatalu(.a(FSrcAE), .b(FSrcBE), .ALUControl(ALUControlE), .f(floatResultE), .flags(floatFlags));
  // MEMORY
  $display("  M: RdM=%0d ALUResultM=%h WriteDataM=%h RegWriteM=%b ALUPickerM=%b FloatRegWriteM=%b",
           dp.RdM, dp.ALUResultM, dp.WriteDataM, cr.RegWriteM, cr.ALUPickerM, cr.FloatRegWriteM);

  // WRITEBACK
  $display("  W: RdW=%0d ResultW=%h RegWriteW=%b ALUPickerW=%b FloatRegWriteW=%b",
           dp.RdW, dp.ResultW, cr.RegWriteW, cr.ALUPickerW, cr.FloatRegWriteW);

  //HAZARD
  $display("  HAZARD: StallF=%b StallD=%b FlushD=%b FlushE=%b ForwardAE=%b ForwardBE=%b, FloatForwardAE=%b, FloatForwardBE=%b",
           h.StallF, h.StallD, h.FlushD, h.FlushE, h.ForwardAE, h.ForwardBE, h.FloatForwardAE, h.FloatForwardBE);

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
