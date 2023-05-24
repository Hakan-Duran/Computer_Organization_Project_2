`timescale 1ns / 1ps

module register #(parameter N=2)(clk, enable, funsel, load, Q_out);
input clk;
input enable;
input [1:0] funsel;
input [N-1:0] load;
output reg [N-1:0] Q_out;

always @(posedge clk) begin
    if (enable) begin
      case (funsel)
         2'b00 : Q_out <= {N{1'b0}} ;
         2'b01 : Q_out <= load ;
         2'b10 : Q_out <= Q_out - {{(N-1){1'b0}}, 1'b1} ;
         2'b11 : Q_out <= Q_out + {{(N-1){1'b0}}, 1'b1} ;
         default : Q_out <= load ;
      endcase
     end
end
endmodule

module ir (
   clk, data, enable, funsel, lh, irout
);

input clk;
input [7:0] data;
input [1:0] funsel;
input enable;
input lh;
output reg [15:0] irout;

always @(posedge clk) begin
   if (enable) begin
      case (funsel)
         2'b00 : irout <= 16'b0; 
         2'b01 : begin 
            if (!lh) irout[7:0] <= data ;
            else irout[15:8] <= data ;
         end
         2'b10 : irout <= irout - {15'b0, 1'b1} ;
         2'b11 : irout <= irout + {15'b0, 1'b1} ;
      endcase
   end
end
   
endmodule

module mux_2_1(
    input [1:0] Data,
    input [0:0] sel,
    output [0:0] C
    );
    
    assign C = (~sel & Data[0]) | (sel & Data[1]);
    
endmodule

module mux_4_1 (
   input [3:0] Data,
   input [1:0] sel,
   output [0:0] out
);

   wire a1, a2;

   mux_2_1 mu1 (.Data(Data[1:0]), .sel(sel[0]), .C(a1));
   mux_2_1 mu2 (.Data(Data[3:2]), .sel(sel[0]), .C(a2));

   mux_2_1 mu3 (.Data({a2, a1}), .sel(sel[1]), .C(out));
   
endmodule

module mux_8_1(
    input [7:0] Data,
    input [2:0] sel,
    output [0:0] out
);

    wire sel1;
    wire sel2;
    wire sel3;
    wire sel4;
    wire sel5;
    wire sel6;

    mux_2_1 m1 (.Data(Data[1:0]), .sel(sel[0]), .C(sel1));
    mux_2_1 m2 (.Data(Data[3:2]), .sel(sel[0]), .C(sel2));
    mux_2_1 m3 (.Data(Data[5:4]), .sel(sel[0]), .C(sel3));
    mux_2_1 m4 (.Data(Data[7:6]), .sel(sel[0]), .C(sel4));
    
    mux_2_1 m5 (.Data({sel2, sel1}), .sel(sel[1]), .C(sel5));
    mux_2_1 m6 (.Data({sel4, sel3}), .sel(sel[1]), .C(sel6));
    
    mux_2_1 m7 (.Data({sel6, sel5}), .sel(sel[2]), .C(out));

endmodule

module reg8_8 (
   clk, load, o1sel, o2sel, funsel, rsel, tsel, o1, o2
);
   
   input clk;
   input [7:0] load;
   input [2:0] o1sel;
   input [2:0] o2sel;
   input [1:0] funsel;
   input [3:0] rsel;
   input [3:0] tsel;
   output [7:0] o1;
   output [7:0] o2;

   wire [7:0] w0, w1, w2, w3, w4, w5, w6, w7 ;

   register#(8) r1 (.clk(clk), .enable(rsel[3]), .funsel(funsel), .load(load), .Q_out(w4)) ; 
   register#(8) r2 (.clk(clk), .enable(rsel[2]), .funsel(funsel), .load(load), .Q_out(w5)) ; 
   register#(8) r3 (.clk(clk), .enable(rsel[1]), .funsel(funsel), .load(load), .Q_out(w6)) ; 
   register#(8) r4 (.clk(clk), .enable(rsel[0]), .funsel(funsel), .load(load), .Q_out(w7)) ;

   register#(8) t1 (.clk(clk), .enable(tsel[3]), .funsel(funsel), .load(load), .Q_out(w0)) ; 
   register#(8) t2 (.clk(clk), .enable(tsel[2]), .funsel(funsel), .load(load), .Q_out(w1)) ; 
   register#(8) t3 (.clk(clk), .enable(tsel[1]), .funsel(funsel), .load(load), .Q_out(w2)) ; 
   register#(8) t4 (.clk(clk), .enable(tsel[0]), .funsel(funsel), .load(load), .Q_out(w3)) ; 

   mux_8_1 o10mux (.Data({w7[0],w6[0],w5[0],w4[0],w3[0],w2[0],w1[0],w0[0]}), .sel(o1sel), .out(o1[0]));
   mux_8_1 o11mux (.Data({w7[1],w6[1],w5[1],w4[1],w3[1],w2[1],w1[1],w0[1]}), .sel(o1sel), .out(o1[1]));
   mux_8_1 o12mux (.Data({w7[2],w6[2],w5[2],w4[2],w3[2],w2[2],w1[2],w0[2]}), .sel(o1sel), .out(o1[2]));
   mux_8_1 o13mux (.Data({w7[3],w6[3],w5[3],w4[3],w3[3],w2[3],w1[3],w0[3]}), .sel(o1sel), .out(o1[3]));
   mux_8_1 o14mux (.Data({w7[4],w6[4],w5[4],w4[4],w3[4],w2[4],w1[4],w0[4]}), .sel(o1sel), .out(o1[4]));
   mux_8_1 o15mux (.Data({w7[5],w6[5],w5[5],w4[5],w3[5],w2[5],w1[5],w0[5]}), .sel(o1sel), .out(o1[5]));
   mux_8_1 o16mux (.Data({w7[6],w6[6],w5[6],w4[6],w3[6],w2[6],w1[6],w0[6]}), .sel(o1sel), .out(o1[6]));
   mux_8_1 o17mux (.Data({w7[7],w6[7],w5[7],w4[7],w3[7],w2[7],w1[7],w0[7]}), .sel(o1sel), .out(o1[7]));

   mux_8_1 o20mux (.Data({w7[0],w6[0],w5[0],w4[0],w3[0],w2[0],w1[0],w0[0]}), .sel(o2sel), .out(o2[0]));
   mux_8_1 o21mux (.Data({w7[1],w6[1],w5[1],w4[1],w3[1],w2[1],w1[1],w0[1]}), .sel(o2sel), .out(o2[1]));
   mux_8_1 o22mux (.Data({w7[2],w6[2],w5[2],w4[2],w3[2],w2[2],w1[2],w0[2]}), .sel(o2sel), .out(o2[2]));
   mux_8_1 o23mux (.Data({w7[3],w6[3],w5[3],w4[3],w3[3],w2[3],w1[3],w0[3]}), .sel(o2sel), .out(o2[3]));
   mux_8_1 o24mux (.Data({w7[4],w6[4],w5[4],w4[4],w3[4],w2[4],w1[4],w0[4]}), .sel(o2sel), .out(o2[4]));
   mux_8_1 o25mux (.Data({w7[5],w6[5],w5[5],w4[5],w3[5],w2[5],w1[5],w0[5]}), .sel(o2sel), .out(o2[5]));
   mux_8_1 o26mux (.Data({w7[6],w6[6],w5[6],w4[6],w3[6],w2[6],w1[6],w0[6]}), .sel(o2sel), .out(o2[6]));
   mux_8_1 o27mux (.Data({w7[7],w6[7],w5[7],w4[7],w3[7],w2[7],w1[7],w0[7]}), .sel(o2sel), .out(o2[7]));

endmodule

module arf (
   clk, load, outasel, outbsel, funsel, rsel, outa, outb
);
   input clk;
   input [7:0] load;
   input [1:0] outasel;
   input [1:0] outbsel;
   input [1:0] funsel;
   input [3:0] rsel;
   output [7:0] outa;
   output [7:0] outb;

   wire [7:0] w0, w1, w2, w3; 
    //rsel 4 bits: 1   1   1       1
    //             AR  SP  PCPrev  PC


   register#(8) ar (.clk(clk), .enable(rsel[3]), .funsel(funsel), .load(load), .Q_out(w0)) ; 
   register#(8) sp (.clk(clk), .enable(rsel[2]), .funsel(funsel), .load(load), .Q_out(w1)) ; 
   register#(8) pcp (.clk(clk), .enable(rsel[1]), .funsel(funsel), .load(load), .Q_out(w2)) ; 
   register#(8) pc (.clk(clk), .enable(rsel[0]), .funsel(funsel), .load(load), .Q_out(w3)) ;

   mux_4_1 a1 (.Data({w3[0],w2[0],w1[0],w0[0]}), .sel(outasel), .out(outa[0]));
   mux_4_1 a2 (.Data({w3[1],w2[1],w1[1],w0[1]}), .sel(outasel), .out(outa[1]));
   mux_4_1 a3 (.Data({w3[2],w2[2],w1[2],w0[2]}), .sel(outasel), .out(outa[2]));
   mux_4_1 a4 (.Data({w3[3],w2[3],w1[3],w0[3]}), .sel(outasel), .out(outa[3]));
   mux_4_1 a5 (.Data({w3[4],w2[0],w1[4],w0[4]}), .sel(outasel), .out(outa[4]));
   mux_4_1 a6 (.Data({w3[5],w2[5],w1[5],w0[5]}), .sel(outasel), .out(outa[5]));
   mux_4_1 a7 (.Data({w3[6],w2[6],w1[6],w0[6]}), .sel(outasel), .out(outa[6]));
   mux_4_1 a8 (.Data({w3[7],w2[7],w1[7],w0[7]}), .sel(outasel), .out(outa[7]));

   mux_4_1 b1 (.Data({w3[0],w2[0],w1[0],w0[0]}), .sel(outbsel), .out(outb[0]));
   mux_4_1 b2 (.Data({w3[1],w2[1],w1[1],w0[1]}), .sel(outbsel), .out(outb[1]));
   mux_4_1 b3 (.Data({w3[2],w2[2],w1[2],w0[2]}), .sel(outbsel), .out(outb[2]));
   mux_4_1 b4 (.Data({w3[3],w2[3],w1[3],w0[3]}), .sel(outbsel), .out(outb[3]));
   mux_4_1 b5 (.Data({w3[4],w2[0],w1[4],w0[4]}), .sel(outbsel), .out(outb[4]));
   mux_4_1 b6 (.Data({w3[5],w2[5],w1[5],w0[5]}), .sel(outbsel), .out(outb[5]));
   mux_4_1 b7 (.Data({w3[6],w2[6],w1[6],w0[6]}), .sel(outbsel), .out(outb[6]));
   mux_4_1 b8 (.Data({w3[7],w2[7],w1[7],w0[7]}), .sel(outbsel), .out(outb[7]));

endmodule



// module flag_reg (
//     clk,
//     load,
//     cin
// );
//     input clk;
//     input [3:0] load;
//     output cin;
//     wire [3:0] out;
//     register#(4) r (.clk(clk), .enable(1'b1), .funsel(2'b01), .load(load), .Q_out(out));

//     assign cin = out[2];

// endmodule


module alu (
    input [7:0] A,
    input [7:0] B,

    input [3:0] Funsel,
    output reg [3:0] Flag,
    output reg [7:0] OutALU
);


reg [8:0] out;
reg [8:0] A_9;
reg [8:0] B_9;
reg [8:0] B_comp_9;


always @(*) begin

A_9 = A;
B_9 = B;
B_comp_9 = ~B;
A_9[8] = 0;
B_9[8] = 0;
B_comp_9[8] = 0;

    case (Funsel)
        4'b0000 : begin 
            OutALU = A; 
            Flag[1] = OutALU[7];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0001 : begin
            OutALU = B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0010 : begin
            OutALU = ~A;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0011 : begin
            OutALU = ~B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0100 : begin
            out = A_9 + B_9;
            if(Flag[2] === 1) begin
                out = out+9'b000000001;
            end
            OutALU = out[7:0];
            Flag[0] = (A[7]&B[7])^OutALU[7];
            Flag[1] = OutALU[7];
            Flag[2] = out[8];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0101 : begin
            out = A_9 + B_comp_9 + 9'b000000001;
            OutALU = out[7:0];
            Flag[0] = (A[7]^B[7])&(A[7]^OutALU[7]);
            Flag[1] = OutALU[7];
            Flag[2] = out[8];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b0110 : begin
                out = A + B_comp_9 + 9'b000000001;
                OutALU = out[7:0];
                Flag[0] = (A[7]^B[7])&(A[7]^OutALU[7]);
                Flag[1] = OutALU[7];
                Flag[2] = out[8];
                if(OutALU === 8'b00000000) begin
                        Flag[3] = 1;
                    end
                else begin
                    Flag[3] = 0;
                    end
                if(~Flag[0]&~Flag[1]&~Flag[3]) begin
                        OutALU = A;
                    end
                else if(Flag[0]&~A[7]) begin
                        OutALU = A;
                    end
                else begin
                        OutALU = 8'b00000000;
                end
            end
        4'b0111 : begin
            OutALU = A&B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1000 : begin
            OutALU = A|B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1001 : begin
            OutALU = ~(A&B);
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1010 : begin
            OutALU = A^B;
            Flag[1] = OutALU[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1011 : begin
            OutALU = A<<1;
            Flag[1] = OutALU[7];
            Flag[2] = A[7];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1100 : begin
            OutALU = A>>1;
            Flag[1] = OutALU[7];
            Flag[2] = A[0];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1101 : begin
            OutALU = A<<1;
            Flag[0] = A[7]^OutALU[7];
            Flag[1] = OutALU[7];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1110 : begin
            OutALU = A>>1;
            OutALU[7] = A[7]; 
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        4'b1111 : begin
            OutALU = A>>1;
            Flag[2] = A[0];
            OutALU[7] = Flag[2];
            Flag[1] = OutALU[7];
            if(OutALU === 8'b00000000) begin
                    Flag[3] = 1;
                end
            else begin
                Flag[3] = 0;
                end
            end
        default : begin
            OutALU = 8'b00000000;
            Flag = 4'b0000;
        end
    endcase
end
endmodule

// module aluPlusFlagReg(input clock,input[7:0] A,input[7:0] B,input[3:0]funsel,output[7:0] outALU);
// wire[3:0] flag;
// wire cin;
// flag_reg FlagReg(clock,flag,cin);
// alu arLogUn( A, B, cin, funsel, flag, outALU);



// endmodule



//last question **************************************
//given module for memory
module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1  //read from memory, write to memory 
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
);
    //Declaration of the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule


module twoToOneMuxOf8bits(input selector, input[7:0] in0, input[7:0] in1, output reg[7:0] out);

always @(*) begin
   case (selector)
      1'b0 : out= in0;
      1'b1 : out= in1;
      default: assign out= in0;
   endcase
end
   
endmodule


module fourToOneMuxOf8bits(input[1:0] selector, input[7:0] in0, input[7:0] in1,input[7:0] in2,input[7:0] in3, output reg[7:0] out);

always @(*) begin
   case (selector)
      2'b00 :  out= in0;
      2'b01 :  out= in1;
      2'b10 :  out= in2;
      2'b11 :  out= in3;
      default: out= in0;
   endcase
end
   
endmodule







module ALU_System (
    input [1:0] ARF_OutCSel,
    input[1:0] ARF_OutDSel,
    input [1:0] IR_Funsel,
    input [1:0] ARF_FunSel,
    input [1:0] RF_FunSel,
    input [3:0] ALU_FunSel,


    input [3:0] RF_RSel,
    input [3:0] ARF_RegSel,
    input Clock,
    input Mem_WR,
    input Mem_CS,
    input IR_Enable,
    input IR_LH,
    input [1:0] MuxASel,
    input [1:0] MuxBSel,
    input MuxCSel,
    input [2:0]RF_OutASel, 
    input [2:0]RF_OutBSel, 
    input [3:0]RF_TSel,

   
    output wire [7:0] AOut,//rf-muxC
    output wire [7:0] BOut,//rf-alu
    output wire [7:0] ALUOut,
    output wire [3:0] ALUOutFlag,
    output wire [7:0] ARF_AOut,//arf -muxA
    output wire [7:0] Address,//arf- memory adress
    output wire [7:0] MemoryOut, //memory - IR- muxA
    output wire [7:0] MuxAOut, //muxA- rf
    output wire [7:0] MuxBOut,//muxB-arf
    output wire [7:0] MuxCOut,//muxC-alu
    output wire [15:0] IROut //direct IR output
    );


    wire [7:0] IROut_LSBs; //less significant bits of IROut
    wire [7:0] IROut_MSBs; //most significant bits of IROut



    arf ARF(Clock,MuxBOut, ARF_OutCSel, ARF_OutDSel,ARF_FunSel,ARF_RegSel,ARF_AOut,Address);
    Memory MEMORY(Address, ALUOut, Mem_WR, Mem_CS, Clock, MemoryOut);

    ir IR(Clock, MemoryOut, IR_Enable,IR_Funsel,IR_LH, IROut); 
    assign IROut_MSBs =IROut[15:8];
    assign IROut_LSBs=IROut[7:0];


    fourToOneMuxOf8bits muxA( MuxASel, ALUOut, MemoryOut,  IROut_LSBs, ARF_AOut, MuxAOut);
    fourToOneMuxOf8bits muxB(MuxBSel, ALUOut,MemoryOut,IROut_LSBs, ARF_AOut, MuxBOut);

    reg8_8 Register_File(Clock, MuxAOut, RF_OutASel, RF_OutBSel, RF_FunSel, RF_RSel, RF_TSel, AOut, BOut);

    twoToOneMuxOf8bits muxC(MuxCSel, AOut, ARF_AOut, MuxCOut); //0=> AOut, 1=> ARF_AOut
    alu ALU(MuxCOut,BOut,ALU_FunSel,ALUOutFlag,ALUOut); 


endmodule

//before this section was in project 1





module Counter(input wire clock, input wire reset, output reg [3:0] register_counter);
    always @(posedge clock or posedge reset) begin
        if (reset)
        begin 
            register_counter=4'b1111;
        end
        else
        begin
            register_counter<= #0.25 register_counter+4'b0001;
        end
    end

endmodule




module Control_Unit_Combined_With_ALU_System (input Clock, /*input reset_timing,*/ 


    // output wires for control purposes
    output wire [7:0] AOut,
    output wire [7:0] BOut,
    output wire [7:0] ALUOut,
    output wire [3:0] ALUOutFlag,
    output wire [7:0] ARF_AOut,
    output wire [7:0] Address,
    output wire [7:0] MemoryOut,
    output wire [7:0] MuxAOut,
    output wire [7:0] MuxBOut,
    output wire [7:0] MuxCOut,
    output wire [15:0] IROut,
    output wire [3:0] timing_signal,


    output reg[3:0] ins_opcode,
    output reg [1:0] ARF_OutCSel,
    output reg[1:0] ARF_OutDSel,
    output reg [1:0] IR_Funsel,
    output reg [1:0] ARF_FunSel,
    output reg [1:0] RF_FunSel,
    output reg [3:0] ALU_FunSel,
    output reg [3:0] RF_RSel,
    output reg [3:0] ARF_RegSel,
    output reg Mem_WR,
    output reg Mem_CS,
    output reg IR_Enable,
    output reg IR_LH,
    output reg [1:0] MuxASel,
    output reg [1:0] MuxBSel,
    output reg MuxCSel,
    output reg [2:0] RF_OutASel,
    output reg [2:0] RF_OutBSel,
    output reg [3:0] RF_TSel




);//don't forget to reset counter at the begining and return back to normal condition!!!!
    reg reset_timing_signal; 
    
    Counter counter(Clock, (reset_timing_signal /*| reset_timing*/),timing_signal);


    // //wires for ALU_system
    // //input wires
    // reg [1:0] ARF_OutCSel;    //ARF_OutASel 
    // reg[1:0] ARF_OutDSel;     //ARF_OutBSel
    // reg [1:0] IR_Funsel;
    // reg [1:0] ARF_FunSel;
    // reg [1:0] RF_FunSel;
    // reg [3:0] ALU_FunSel;
    // reg [3:0] RF_RSel;
    // reg [3:0] ARF_RegSel;
    // // rsel   places: 3   2   1       0
    // // values 4 bits: 1   1   1       1
    // //                AR  SP  PCPrev  PC
    
    
    // reg Mem_WR;
    // reg Mem_CS;
    // reg IR_Enable;
    // reg IR_LH;//0=> (7-0), 1=> (15-8)
    // reg [1:0] MuxASel;
    // reg [1:0] MuxBSel;
    // reg MuxCSel; //0=> AOut(RF), 1=> ARF_AOut
    // reg [2:0] RF_OutASel;
    // reg [2:0] RF_OutBSel;
    // reg [3:0] RF_TSel;
    // //end of input wires




//    //output wires
//     wire [7:0] AOut;//rf-muxC
//     wire [7:0] BOut;//rf-alu
//     wire [7:0] ALUOut;
//     wire [3:0] ALUOutFlag;
//     wire [7:0] ARF_AOut;//arf -muxA
//     wire [7:0] Address;//arf- memory adress
//     wire [7:0] MemoryOut; //memory - IR- muxA
//     wire [7:0] MuxAOut; //muxA- rf
//     wire [7:0] MuxBOut;//muxB-arf
//     wire [7:0] MuxCOut;//muxC-alu
//     wire [15:0] IROut; //direct IR output
//     //end of output wires
//     //end of wires for ALU_system





  

    //not sure
    // always @(*) begin
    //     if((ARF_RegSel==4'b0001) && (ARF_FunSel==2'b11) || (timing_signal==4'b1111))begin
    //         ins_opcode<= #5 IROut[15:12];
    //     end
    // end


    
    

    //figure 2 mode: (if adressing mode is N/A from the table)
    wire[3:0] ins_dstreg = IROut[11:8];
    wire [3:0] ins_sreg1= IROut[7:4];
    wire [3:0] ins_sreg2= IROut[3:0];

    //figure 1 mode: (if adressing mode is IM, D in the table)

    wire ins_addressing_mode =IROut[10];
    wire [1:0] ins_rsel= IROut[9:8];
    wire [7:0] ins_adress= IROut[7:0];



    ALU_System sys(ARF_OutCSel,ARF_OutDSel,IR_Funsel,ARF_FunSel,RF_FunSel,ALU_FunSel,RF_RSel,
    ARF_RegSel,Clock,Mem_WR,Mem_CS,IR_Enable,IR_LH,MuxASel,MuxBSel,MuxCSel,RF_OutASel, RF_OutBSel,
     RF_TSel,AOut, BOut, ALUOut, ALUOutFlag, ARF_AOut, Address, MemoryOut,  MuxAOut,  MuxBOut, MuxCOut,IROut 
    );

      
    // ********** This statments will be used later for output 
        // begin
        //     if (ins_dstreg[2]==1'b0)//write to rf
        //     begin 
        //         case (ins_dstreg[1:0]) //which register will be used
        //             2'b00: //R1
        //                 RF_RSel<= 4'b1000;
            
        //             2'b01: //R2
        //                 RF_RSel<= 4'b0100;

        //             2'b10: //R3
        //                 RF_RSel<= 4'b0010;

        //             2'b11: //R4
        //                 RF_RSel<= 4'b0001;
                    
        //         endcase
        //         RF_FunSel=2'b01; //open to load

        //     end
        //     else begin//write to arf 
                    
        //         case (ins_dstreg[1:0]) //which register will be used
        //             2'b00: //SP
        //                 ARF_RegSel<= 4'b0100;
            
        //             2'b01: //AR
        //                 ARF_RegSel<= 4'b1000;

        //             2'b10: //PC
        //                 ARF_RegSel<= 4'b0001;

        //             2'b11: //PC
        //                 ARF_RegSel<= 4'b0001;
                    
        //         endcase
        //         ARF_FunSel=2'b01; //open to load

        //      end 
        
        // end


    reg delay;

    //make everything 0
    initial begin
        Mem_WR<=0;
        
        ARF_FunSel<=2'b00;
        RF_FunSel<=2'b00;
        RF_RSel<=4'b1111;
        ARF_RegSel<=4'b1111;
        RF_TSel<=4'b1111;
        reset_timing_signal<=1;
        delay<=1;

        // ARF_OutCSel<=2'b11;
        // ARF_OutDSel<=2'b11;
        // RF_OutASel<=3'b111;
        // RF_OutBSel<=3'b111;
        
        #5;
        RF_RSel<=4'b0000;
        ARF_RegSel<=4'b0000;
        RF_TSel<=4'b0000;
        reset_timing_signal<=0;
        delay<=0;
     end



    always @(posedge reset_timing_signal) begin
        if(!delay) begin
            ARF_RegSel<= #5 4'b0000; //making sure ARF_RegSel deactiveted for unintended writings
            IR_Enable<= #5 0; 
            RF_RSel<= #5 4'b0000;
            RF_TSel<= #5 4'b0000;//making sure RF_TSel deactiveted for unintended writings
            reset_timing_signal<= #5 0;//be sure that timing signal doesn't reset in every cycle
        end
    end


    always @(timing_signal) begin
        ARF_RegSel<= #10 4'b0000; //making sure ARF_RegSel deactiveted for unintended writings
        IR_Enable<= #10 0; 
        RF_RSel<= #10 4'b0000;
        RF_TSel<= #10 4'b0000;//making sure RF_TSel deactiveted for unintended writings
    end
 

    //operations
    always @(*) begin  //registers have internal clock mechanisms, don't implement yours
        Mem_CS<=0;
        ins_opcode<=IROut[15:12];
        //opcode and timing signal condtions
        //don't forget to count from 0
        if (

            ((ins_opcode == 4'h0 )&& (timing_signal == 4'b0110 ))||
            ((ins_opcode == 4'h1 )&& (timing_signal == 4'b0110  ))||
            ((ins_opcode == 4'h2 )&& (timing_signal == 4'b0110  ))||
            ((ins_opcode == 4'h3 )&& (timing_signal == 4'b0110  ))||
            ((ins_opcode == 4'h4 )&& (timing_signal == 4'b0110  ))||
            ((ins_opcode == 4'h5 )&& (timing_signal == 4'b0110 ))||
            ((ins_opcode == 4'h6 )&& (timing_signal == 4'b0110 ))||
            ((ins_opcode == 4'h7 )&& (timing_signal == 4'b1000 ))||
            ((ins_opcode == 4'h8 )&& (timing_signal == 4'b1000  ))||
            ((ins_opcode == 4'h9 )&& (timing_signal == 4'b0100  ))||
            ((ins_opcode == 4'hA )&& (timing_signal == 4'b0100  ))||
            ((ins_opcode == 4'hB )&& (timing_signal == 4'b0110 )) ||
            ((ins_opcode == 4'hC )&& (timing_signal == 4'b0100 )) ||
            ((ins_opcode == 4'hD )&& (timing_signal == 4'b0100 ))||
            ((ins_opcode == 4'hE )&& (timing_signal == 4'b0101))||
            ((ins_opcode == 4'hF )&& (timing_signal == 4'b0101 ))
            
         ) begin
             reset_timing_signal <= 1'b1; //counter is zeroed.
        end 

        // if((reset_timing_signal==1'b1) ||  (timing_signal==4'b1111))begin
        //     ins_opcode_for_reset<= #10 ins_opcode;
        // end
        
        // ins_opcode<= #5 IROut[15:12];
        IR_Enable<=0;


        if(timing_signal==4'b0000) begin //start of the fetch cycle 
            //IR(7-0)<-M[PC]
            ARF_OutDSel<=2'b 11; //PC will be given as adress to memory
            Mem_WR<=0;    //read from memory
            IR_Enable<=1; //activate IR
            IR_Funsel<=2'b 01; //open load
            IR_LH<=0;  //IR(7-0) selected
            ARF_RegSel <=4'b0000;


         end
        else if(timing_signal==4'b0001) begin //2nd phase of fetch cycle
            //IR(15-8)<-M[PC],  PC<-PC+1
            IR_Enable<=0;
            ARF_FunSel<=2'b11; //increment by 1
            ARF_RegSel <=4'b0001; //open PC
           

           
            

         end
        else if (timing_signal==4'b0010)begin //3rd and last phase of the fetch cycle
            //PC<-PC+1

            // ARF_FunSel<=2'b11; //increment by 1
            // ARF_RegSel <=4'b0001; //open PC
            ARF_OutDSel<=2'b 11; //PC will be given as adress to memory
            Mem_WR<=0;    //read from memory
            IR_Enable<=1; //activate IR
            IR_Funsel<=2'b 01; //open load
            IR_LH<=1;  //IR(15-8) selected


            ARF_FunSel<=2'b11; //increment by 1
            ARF_RegSel <=4'b0001; //open PC

         end
         
        //IR is ready to use

        //register selections for output 
        
        else if(timing_signal==4'b0011)begin
            ARF_RegSel <=4'b0000;
  
            if ((ins_opcode == 4'h9) || (ins_opcode == 4'hA) || (ins_opcode == 4'hC) || (ins_opcode == 4'hD) || (ins_opcode == 4'hE)|| (ins_opcode == 4'hF)) //in these opcodes figure 1 order will be used
            begin 
                //selection of AR or M[AR] not decided in here, it varies a lot depending on opcode, 
                //so in the opcode address mode selection will be made.
                //in here AR<-IR(7-0) operation made
                //also rsel selection in here also depends on the state (being input or output), so they will also be considered in opcodes


                //NORMALLY This operations should be done as below prevously, but depending on the operation of these 4 opcode, 
                //AR value may not be needed to be updated, so you will do it on the opcodes   EMRE************************* 


                // MuxBSel<=2'b10; //IR(7-0) will go to ARF
                // ARF_RegSel<=4'b1000; // select only AR
                // ARF_FunSel<=2'b01; //load 

                if(ins_opcode == 4'h9) begin
                    IR_Enable<=0;
                    MuxBSel<=2'b10; //IR(7-0) will go to ARF
                    ARF_RegSel<=4'b0001; // select only PC
                    ARF_FunSel<=2'b01; //load 
                   
                end

                else if(ins_opcode == 4'hA) begin
                    IR_Enable<=0;
                if(ALUOutFlag[3]==1'b0) begin
                    MuxBSel<=2'b10; //IR(7-0) will go to ARF
                    ARF_RegSel<=4'b0001; // select only PC
                    ARF_FunSel<=2'b01; //load 
                    end
                end

                else if(ins_opcode == 4'hC) begin
                    IR_Enable<=0;
                    if(ins_addressing_mode==1'b0) begin //immediate addressing
                        MuxASel<=2'b10; //selects IROut
                        RF_FunSel<= 2'b01; //open load for RF
                        case (ins_rsel)
                        2'b00: begin
                        RF_RSel<=4'b1000;   //R1 is chosen
                        end
                        2'b01: begin
                        RF_RSel<=4'b0100;   //R2 is chosen
                        end
                        2'b10: begin
                        RF_RSel<=4'b0010;   //R3 is chosen
                        end
                        2'b11: begin
                        RF_RSel<=4'b0001;   //R4 is chosen
                        end
                        endcase
                    end
                    else begin //direct addressing
                        ARF_OutDSel<=2'b 00; //AR will be given as adress to memory
                        Mem_WR<=0;    //read from memory
                        MuxASel<=2'b01; // selects memory output in MUX A
                        RF_FunSel<=2'b01; //open load for RF
                        case (ins_rsel)
                        2'b00: begin
                        RF_RSel<=4'b1000;   //R1 is chosen
                        end
                        2'b01: begin
                        RF_RSel<=4'b0100;   //R2 is chosen
                        end
                        2'b10: begin
                        RF_RSel<=4'b0010;   //R3 is chosen
                        end
                        2'b11: begin
                        RF_RSel<=4'b0001;   //R4 is chosen
                        end
                        endcase    
                    end
                end
                else if(ins_opcode == 4'hD && timing_signal==4'b0011) begin
                    IR_Enable<=0;
                    ARF_OutDSel<=2'b 00; //AR will be given as adress to memory
                    case (ins_rsel)
                    2'b00: begin
                    RF_OutASel<=3'b100;    //R1 is sent to MUXC
                    end
                    2'b01: begin
                    RF_OutASel<=3'b101;    //R2 is sent to MUXC
                    end
                    2'b10: begin
                    RF_OutASel<=3'b110;    //R3 is sent to MUXC
                    end
                    2'b11: begin
                    RF_OutASel<=3'b111;    //R4 is sent to MUXC
                    end
                    endcase
                    MuxCSel<=0; //RF's output is sent to alu
                    ALU_FunSel<=4'b0000; //RF is sent to memory
                    Mem_WR<=1;    //write to from memory
                   

                end
                else if(ins_opcode == 4'hE && timing_signal==4'b0011) begin
                    IR_Enable<=0;
                    ARF_OutDSel<=2'b 01;  // SP is given as address to memory
                    Mem_WR<=0;    //read from memory
                    MuxASel<=2'b01; //selects memory for MUX A
                    RF_FunSel<= 2'b01; //open load for RF
                    case (ins_rsel)
                    2'b00: begin
                    RF_RSel<=4'b1000;   //R1 is chosen
                    end
                    2'b01: begin
                    RF_RSel<=4'b0100;   //R2 is chosen
                    end
                    2'b10: begin
                    RF_RSel<=4'b0010;   //R3 is chosen
                    end
                    2'b11: begin
                    RF_RSel<=4'b0001;   //R4 is chosen
                    end
                    endcase

                end
                else if(ins_opcode == 4'hF && timing_signal==4'b0011) begin
                    IR_Enable<=0;
                    ARF_RegSel<= 4'b0100; // SP is selected
                    ARF_OutDSel<=2'b01;  // SP is given as address to memory
                    case (ins_rsel)
                    2'b00: begin
                    RF_RSel<=4'b1000;   //R1 is chosen
                    end
                    2'b01: begin
                    RF_RSel<=4'b0100;   //R2 is chosen
                    end
                    2'b10: begin
                    RF_RSel<=4'b0010;   //R3 is chosen
                    end
                    2'b11: begin
                    RF_RSel<=4'b0001;   //R4 is chosen
                    end
                    endcase
                    MuxCSel<=1'b0; // RF is selected
                    ALU_FunSel <= 4'b0000; //register value is written to the memory
                    Mem_WR<=1;    //write to memory
                end
             end



            else begin //in other opcodes figure 2 order will be used
                //there is 3 situation for input. Both can be in RF, one form RF one from ARF, both can be in ARF
                case({ins_sreg1[2],ins_sreg2[2]})

                    2'b00: //both are in RF 
                        begin
                            //0=> AOut, 1=> ARF_AOut
                            MuxCSel<=0; //RF's output
                            RF_OutASel<={1'b1,ins_sreg1[1:0]};//RF output selection
                            RF_OutBSel<={1'b1,ins_sreg2[1:0]};//RF output selection
                        end
                    2'b10: //1st ARF, 2nd RF
                        begin
                        MuxCSel<=1;   //ARF's output
                        RF_OutBSel<={1'b1,ins_sreg2[1:0]};//RF output selection
                        
                        case (ins_sreg1[1:0])//ARF output selection
                            2'b00:  //SP selection
                                ARF_OutCSel<= 2'b01;
                            2'b01: //AR selection
                                ARF_OutCSel<= 2'b00;
                            2'b10://PC selection
                                ARF_OutCSel<=2'b11;
                            2'b11://PC selection
                                ARF_OutCSel<=2'b11;
                        endcase 
                        end 

                    2'b01: //1st RF, 2nd ARF (1 more cycle needed)
                        begin//store value of ARF in T1

                            case (ins_sreg2[1:0])//ARF output selection
                                2'b00:  //SP selection
                                    ARF_OutCSel<= 2'b01;
                                2'b01: //AR selection
                                    ARF_OutCSel<= 2'b00;
                                2'b10://PC selection
                                    ARF_OutCSel<=2'b11;
                                2'b11://PC selection
                                    ARF_OutCSel<=2'b11;
                            endcase

                            MuxASel<=2'b11; //output coming from ARF
                            RF_TSel<=4'b1000; //T1 will be used to store value coming from ARF
                            RF_FunSel<= 2'b01; //open load
                        end
                    2'b11: //both are ARF (1 more cycle needed)
                        begin //store value of ARF in T1 (for ALU input B)
                            case (ins_sreg2[1:0])//ARF output selection
                                2'b00:  //SP selection
                                    ARF_OutCSel<= 2'b01;
                                2'b01: //AR selection
                                    ARF_OutCSel<= 2'b00;
                                2'b10://PC selection
                                    ARF_OutCSel<=2'b11;
                                2'b11://PC selection
                                    ARF_OutCSel<=2'b11;
                            endcase

                            MuxASel<=2'b11; //output coming from ARF
                            RF_TSel<=4'b1000; //T1 will be used to store value coming from ARF
                            RF_FunSel<= 2'b01; //open load
                        end
                endcase


          
             end
         end
        

        
        else if((timing_signal==4'b0100) &&  (  {ins_sreg1[2],ins_sreg2[2]} ==  2'b01 )  && 
                 !(  (ins_opcode == 4'h9) || (ins_opcode == 4'hA) || (ins_opcode == 4'hC) || (ins_opcode == 4'hD)  )  )begin  //next cycle of 1st RF, 2nd ARF
            RF_OutBSel<=3'b000; //T1 will be 2nd output 
            MuxCSel<=0; //Rx RF's output
            RF_OutASel<= {1'b1,ins_sreg1[1:0]};//Rx selection
        end
        else if((timing_signal==4'b0100) &&  (  {ins_sreg1[2],ins_sreg2[2]} ==  2'b11 ) &&
                  !(  (ins_opcode == 4'h9) || (ins_opcode == 4'hA) || (ins_opcode== 4'hC) || (ins_opcode == 4'hD)  ))begin  //next cycle of both are ARF
            
            MuxCSel<=1;   //ARF's output for ALU A (sreg 1)

            case (ins_sreg1[1:0])//ARF output selection for sreg1
                2'b00:  //SP selection
                    ARF_OutCSel<= 2'b01;
                2'b01: //AR selection
                    ARF_OutCSel<= 2'b00;
                2'b10://PC selection
                    ARF_OutCSel<=2'b11;
                2'b11://PC selection
                    ARF_OutCSel<=2'b11;
            endcase 
            RF_OutBSel<=3'b000; //T1 will be 2nd output (sreg 2)
        
        
        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h0)) begin // AND
            IR_Enable<=0;
            ALU_FunSel <= 4'b0111 ;
            Mem_WR <= 0;

            begin // write to register

            if (ins_dstreg[2]==1'b0)//write to rf
                begin 
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //R1
                            RF_RSel<= 4'b1000;
                
                        2'b01: //R2
                            RF_RSel<= 4'b0100;

                        2'b10: //R3
                            RF_RSel<= 4'b0010;

                        2'b11: //R4
                            RF_RSel<= 4'b0001;
                        
                    endcase
                    RF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b00; //OutALU
                    MuxBSel <= 2'b10; //IROut (random)
                    ARF_RegSel <= 3'b000; //Do not let ARF to change.

                end

            else 
                begin//write to arf 
                    
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //SP
                            ARF_RegSel<= 4'b0100;
                
                        2'b01: //AR
                            ARF_RegSel<= 4'b1000;

                        2'b10: //PC
                            ARF_RegSel<= 4'b0001;

                        2'b11: //PC
                            ARF_RegSel<= 4'b0001;
                        
                    endcase
                    ARF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b10; //IROut (random)
                    MuxBSel <= 2'b00; //OutALU
                    RF_RSel <= 3'b000; //Do not let RF to change.

                end 
        
            end

           
            
        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h1)) begin // OR
            IR_Enable<=0;
            ALU_FunSel <= 4'b1000 ;
            Mem_WR <= 0;

            begin // write to register

            if (ins_dstreg[2]==1'b0)//write to rf
                begin 
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //R1
                            RF_RSel<= 4'b1000;
                
                        2'b01: //R2
                            RF_RSel<= 4'b0100;

                        2'b10: //R3
                            RF_RSel<= 4'b0010;

                        2'b11: //R4
                            RF_RSel<= 4'b0001;
                        
                    endcase
                    RF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b00; //OutALU
                    MuxBSel <= 2'b10; //IROut (random)
                    ARF_RegSel <= 3'b000; //Do not let ARF to change.

                end

            else 
                begin//write to arf 
                    
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //SP
                            ARF_RegSel<= 4'b0100;
                
                        2'b01: //AR
                            ARF_RegSel<= 4'b1000;

                        2'b10: //PC
                            ARF_RegSel<= 4'b0001;

                        2'b11: //PC
                            ARF_RegSel<= 4'b0001;
                        
                    endcase
                    ARF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b10; //IROut (random)
                    MuxBSel <= 2'b00; //OutALU
                    RF_RSel <= 3'b000; //Do not let RF to change.

                end 
        
            end

            
            
        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h2)) begin // NOT
            IR_Enable<=0;
            ALU_FunSel <= 4'b0010 ;
            Mem_WR <= 0;

            begin // write to register

            if (ins_dstreg[2]==1'b0)//write to rf
                begin 
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //R1
                            RF_RSel<= 4'b1000;
                
                        2'b01: //R2
                            RF_RSel<= 4'b0100;

                        2'b10: //R3
                            RF_RSel<= 4'b0010;

                        2'b11: //R4
                            RF_RSel<= 4'b0001;
                        
                    endcase
                    RF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b00; //OutALU
                    MuxBSel <= 2'b10; //IROut (random)
                    ARF_RegSel <= 3'b000; //Do not let ARF to change.

                end

            else 
                begin//write to arf 
                    
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //SP
                            ARF_RegSel<= 4'b0100;
                
                        2'b01: //AR
                            ARF_RegSel<= 4'b1000;

                        2'b10: //PC
                            ARF_RegSel<= 4'b0001;

                        2'b11: //PC
                            ARF_RegSel<= 4'b0001;
                        
                    endcase
                    ARF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b10; //IROut (random)
                    MuxBSel <= 2'b00; //OutALU
                    RF_RSel <= 3'b000; //Do not let RF to change.

                end 
        
            end

            
            
        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h3)) begin // ADD
            IR_Enable<=0;
            ALU_FunSel <= 4'b0100 ;
            Mem_WR <= 0;

            begin // write to register

            if (ins_dstreg[2]==1'b0)//write to rf
                begin 
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //R1
                            RF_RSel<= 4'b1000;
                
                        2'b01: //R2
                            RF_RSel<= 4'b0100;

                        2'b10: //R3
                            RF_RSel<= 4'b0010;

                        2'b11: //R4
                            RF_RSel<= 4'b0001;
                        
                    endcase
                    RF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b00; //OutALU
                    MuxBSel <= 2'b10; //IROut (random)
                    ARF_RegSel <= 3'b000; //Do not let ARF to change.

                end

            else 
                begin//write to arf 
                    
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //SP
                            ARF_RegSel<= 4'b0100;
                
                        2'b01: //AR
                            ARF_RegSel<= 4'b1000;

                        2'b10: //PC
                            ARF_RegSel<= 4'b0001;

                        2'b11: //PC
                            ARF_RegSel<= 4'b0001;
                        
                    endcase
                    ARF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b10; //IROut (random)
                    MuxBSel <= 2'b00; //OutALU
                    RF_RSel <= 3'b000; //Do not let RF to change.

                end 
        
            end

            
        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h4)) begin // SUB
           IR_Enable<=0;
           ALU_FunSel <= 4'b0101 ;
            Mem_WR <= 0;

            begin // write to register

            if (ins_dstreg[2]==1'b0)//write to rf
                begin 
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //R1
                            RF_RSel<= 4'b1000;
                
                        2'b01: //R2
                            RF_RSel<= 4'b0100;

                        2'b10: //R3
                            RF_RSel<= 4'b0010;

                        2'b11: //R4
                            RF_RSel<= 4'b0001;
                        
                    endcase
                    RF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b00; //OutALU
                    MuxBSel <= 2'b10; //IROut (random)
                    ARF_RegSel <= 3'b000; //Do not let ARF to change.

                end

            else 
                begin//write to arf 
                    
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //SP
                            ARF_RegSel<= 4'b0100;
                
                        2'b01: //AR
                            ARF_RegSel<= 4'b1000;

                        2'b10: //PC
                            ARF_RegSel<= 4'b0001;

                        2'b11: //PC
                            ARF_RegSel<= 4'b0001;
                        
                    endcase
                    ARF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b10; //IROut (random)
                    MuxBSel <= 2'b00; //OutALU
                    RF_RSel <= 3'b000; //Do not let RF to change.

                end 
        
            end

            
            
        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h5)) begin // LSR
            IR_Enable<=0;
            ALU_FunSel <= 4'b1100 ;
            Mem_WR <= 0;

            begin // write to register

            if (ins_dstreg[2]==1'b0)//write to rf
                begin 
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //R1
                            RF_RSel<= 4'b1000;
                
                        2'b01: //R2
                            RF_RSel<= 4'b0100;

                        2'b10: //R3
                            RF_RSel<= 4'b0010;

                        2'b11: //R4
                            RF_RSel<= 4'b0001;
                        
                    endcase
                    RF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b00; //OutALU
                    MuxBSel <= 2'b10; //IROut (random)
                    ARF_RegSel <= 3'b000; //Do not let ARF to change.

                end

            else 
                begin//write to arf 
                    
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //SP
                            ARF_RegSel<= 4'b0100;
                
                        2'b01: //AR
                            ARF_RegSel<= 4'b1000;

                        2'b10: //PC
                            ARF_RegSel<= 4'b0001;

                        2'b11: //PC
                            ARF_RegSel<= 4'b0001;
                        
                    endcase
                    ARF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b10; //IROut (random)
                    MuxBSel <= 2'b00; //OutALU
                    RF_RSel <= 3'b000; //Do not let RF to change.

                end 
        
            end

            
        end


        else if((timing_signal==4'b0101) && (ins_opcode == 4'h6)) begin // LSL
            IR_Enable<=0;
            ALU_FunSel <= 4'b1011 ;
            Mem_WR <= 0;

            begin // write to register

            if (ins_dstreg[2]==1'b0)//write to rf
                begin 
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //R1
                            RF_RSel<= 4'b1000;
                
                        2'b01: //R2
                            RF_RSel<= 4'b0100;

                        2'b10: //R3
                            RF_RSel<= 4'b0010;

                        2'b11: //R4
                            RF_RSel<= 4'b0001;
                        
                    endcase
                    RF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b00; //OutALU
                    MuxBSel <= 2'b10; //IROut (random)
                    ARF_RegSel <= 3'b000; //Do not let ARF to change.

                end

            else 
                begin//write to arf 
                    
                    case (ins_dstreg[1:0]) //which register will be used
                        2'b00: //SP
                            ARF_RegSel<= 4'b0100;
                
                        2'b01: //AR
                            ARF_RegSel<= 4'b1000;

                        2'b10: //PC
                            ARF_RegSel<= 4'b0001;

                        2'b11: //PC
                            ARF_RegSel<= 4'b0001;
                        
                    endcase
                    ARF_FunSel<=2'b01; //open to load
                    MuxASel <= 2'b10; //IROut (random)
                    MuxBSel <= 2'b00; //OutALU
                    RF_RSel <= 3'b000; //Do not let RF to change.

                end 
        
            end

            
            
        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h7)) begin // INC (takes 3 cycle) 1. cycle
            IR_Enable<=0;
            RF_FunSel <= 2'b00; //Clear 
            RF_TSel <= 4'b0001; //Only T4 is enabled.
        end

        else if((timing_signal==4'b0110) && (ins_opcode == 4'h7)) begin // INC (takes 3 cycle) 2. cycle
            IR_Enable<=0;
            RF_OutBSel<=3'b011;
            RF_FunSel <= 2'b11; // Increment
            RF_TSel <= 4'b0001; //Only T4 is enabled. Now T4 is 1.
        end

        else if ((timing_signal==4'b0111) && (ins_opcode == 4'h7)) begin // INC (takes 3 cycle) 3. cycle
            IR_Enable<=0;
            RF_OutBSel<=3'b011;
            ALU_FunSel <= 4'b0100; // SREG1 + 1
            Mem_WR <= 0;
            
            begin // write to register

                if (ins_dstreg[2]==1'b0)//write to rf
                    begin 
                        case (ins_dstreg[1:0]) //which register will be used
                            2'b00: //R1
                                RF_RSel<= 4'b1000;
                    
                            2'b01: //R2
                                RF_RSel<= 4'b0100;

                            2'b10: //R3
                                RF_RSel<= 4'b0010;

                            2'b11: //R4
                                RF_RSel<= 4'b0001;
                            
                        endcase
                        RF_FunSel<=2'b01; //open to load
                        MuxASel <= 2'b00; //OutALU
                        MuxBSel <= 2'b10; //IROut (random)
                        ARF_RegSel <= 3'b000; //Do not let ARF to change.

                    end

                else 
                    begin//write to arf 
                        
                        case (ins_dstreg[1:0]) //which register will be used
                            2'b00: //SP
                                ARF_RegSel<= 4'b0100;
                    
                            2'b01: //AR
                                ARF_RegSel<= 4'b1000;

                            2'b10: //PC
                                ARF_RegSel<= 4'b0001;

                            2'b11: //PC
                                ARF_RegSel<= 4'b0001;
                            
                        endcase
                        ARF_FunSel<=2'b01; //open to load
                        MuxASel <= 2'b10; //IROut (random)
                        MuxBSel <= 2'b00; //OutALU
                        RF_RSel <= 3'b000; //Do not let RF to change.

                    end 
        
            end

        

        end

        else if((timing_signal==4'b0101) && (ins_opcode == 4'h8)) begin // DEC (takes 3 cycle) 1. cycle
            IR_Enable<=0;
            RF_FunSel <= 2'b00; //Clear 
            RF_TSel <= 4'b0001; //Only T4 is enabled.

        end

        else if((timing_signal==4'b0110) && (ins_opcode == 4'h8)) begin // DEC (takes 3 cycle) 2. cycle
            RF_OutBSel<=3'b011;
            IR_Enable<=0;
            RF_FunSel <= 2'b11; // Increment
            RF_TSel <= 4'b0001; //Only T4 is enabled. Now T4 is 1.

        end

        else if ((timing_signal==4'b0111) && (ins_opcode == 4'h8)) begin // DEC (takes 3 cycle) 3. cycle
            RF_OutBSel<=3'b011;
            IR_Enable<=0;
            ALU_FunSel <= 4'b0101; // SREG1 - 1
            Mem_WR <= 0;
            
            begin // write to register

                if (ins_dstreg[2]==1'b0)//write to rf
                    begin 
                        case (ins_dstreg[1:0]) //which register will be used
                            2'b00: //R1
                                RF_RSel<= 4'b1000;
                    
                            2'b01: //R2
                                RF_RSel<= 4'b0100;

                            2'b10: //R3
                                RF_RSel<= 4'b0010;

                            2'b11: //R4
                                RF_RSel<= 4'b0001;
                            
                        endcase
                        RF_FunSel<=2'b01; //open to load
                        MuxASel <= 2'b00; //OutALU
                        MuxBSel <= 2'b10; //IROut (random)
                        ARF_RegSel <= 3'b000; //Do not let ARF to change.

                    end

                else 
                    begin//write to arf 
                        
                        case (ins_dstreg[1:0]) //which register will be used
                            2'b00: //SP
                                ARF_RegSel<= 4'b0100;
                    
                            2'b01: //AR
                                ARF_RegSel<= 4'b1000;

                            2'b10: //PC
                                ARF_RegSel<= 4'b0001;

                            2'b11: //PC
                                ARF_RegSel<= 4'b0001;
                            
                        endcase
                        ARF_FunSel<=2'b01; //open to load
                        MuxASel <= 2'b10; //IROut (random)
                        MuxBSel <= 2'b00; //OutALU
                        RF_RSel <= 3'b000; //Do not let RF to change.

                    end 
        
            end

            

        end


        else if((timing_signal==4'b0101) && (ins_opcode == 4'hB)) begin // MOV
            IR_Enable<=0;
            ALU_FunSel <= 4'b0000 ;
            Mem_WR <= 0;

            begin // write to register

                if (ins_dstreg[2]==1'b0)//write to rf
                    begin 
                        case (ins_dstreg[1:0]) //which register will be used
                            2'b00: //R1
                                RF_RSel<= 4'b1000;
                    
                            2'b01: //R2
                                RF_RSel<= 4'b0100;

                            2'b10: //R3
                                RF_RSel<= 4'b0010;

                            2'b11: //R4
                                RF_RSel<= 4'b0001;
                            
                        endcase
                        RF_FunSel<=2'b01; //open to load
                        MuxASel <= 2'b00; //OutALU
                        MuxBSel <= 2'b10; //IROut (random)
                        ARF_RegSel <= 3'b000; //Do not let ARF to change.
                    end

                else 
                    begin//write to arf 
                        
                        case (ins_dstreg[1:0]) //which register will be used
                            2'b00: //SP
                                ARF_RegSel<= 4'b0100;
                    
                            2'b01: //AR
                                ARF_RegSel<= 4'b1000;

                            2'b10: //PC
                                ARF_RegSel<= 4'b0001;

                            2'b11: //PC
                                ARF_RegSel<= 4'b0001;
                            
                        endcase
                        ARF_FunSel<=2'b01; //open to load
                        MuxASel <= 2'b10; //IROut (random)
                        MuxBSel <= 2'b00; //OutALU
                        RF_RSel <= 3'b000; //Do not let RF to change.

                    end 
        
            end

            

         end

        else if(ins_opcode==4'hE && timing_signal==4'b0100) begin
            IR_Enable<=0;
            ARF_FunSel<=  2'b11;  // Increment
            ARF_RegSel<=  4'b0100; // SP is selected
        end

        else if(ins_opcode==4'hF && timing_signal==4'b0100) begin
            IR_Enable<=0;
            ARF_FunSel<=2'b10;  // Decrement
            ARF_RegSel<= 4'b0100; // SP is selected
        end

        
        // if (

        //     ((ins_opcode == 4'h0 )&& (timing_signal == 4'b0110 ))||
        //     ((ins_opcode == 4'h1 )&& (timing_signal == 4'b0110  ))||
        //     ((ins_opcode == 4'h2 )&& (timing_signal == 4'b0110  ))||
        //     ((ins_opcode == 4'h3 )&& (timing_signal == 4'b0110  ))||
        //     ((ins_opcode == 4'h4 )&& (timing_signal == 4'b0110  ))||
        //     ((ins_opcode == 4'h5 )&& (timing_signal == 4'b0110 ))||
        //     ((ins_opcode == 4'h6 )&& (timing_signal == 4'b0110 ))||
        //     ((ins_opcode == 4'h7 )&& (timing_signal == 4'b1000 ))||
        //     ((ins_opcode == 4'h8 )&& (timing_signal == 4'b1000  ))||
        //     ((ins_opcode == 4'h9) && (timing_signal == 4'b0100))||
        //     ((ins_opcode == 4'hA )&& (timing_signal == 4'b0100  ))||
        //     ((ins_opcode == 4'hB )&& (timing_signal == 4'b0110 ))||
        //     ((ins_opcode == 4'hC )&& (timing_signal == 4'b0100 ))||
        //     ((ins_opcode == 4'hD )&& (timing_signal == 4'b0100 ))||
        //     ((ins_opcode == 4'hE )&& (timing_signal == 4'b0101))||
        //     ((ins_opcode == 4'hF )&& (timing_signal == 4'b0101 ))
            
        //  ) begin
        //      reset_timing_signal <= 1'b1; //counter is zeroed.
        // end 






        //********************************************************************************************************
        //DO NOT FORGET TO REMOVE SPECIFCATIONS (LIKE MAKE regsel=0, OR CLOSE WRITING TO MEMORY) FROM MEMORY AND REGISTERS AT THE END,
        //OTHERWISE NEW OPERATIONS MAY OVERWRITE NECESSARY CONTENT

        // if (1)begin     //place will be reconsidered !!!!!!!!!!

            
        //     ARF_RegSel<= #2 4'b0000; //making sure ARF_RegSel deactiveted for unintended writings
        //     reset_timing_signal<= #2 0;//be sure that timing signal doesn't reset in every cycle
        //     IR_Enable<= #2 0; 
        //     RF_RSel<= #2 4'b0000;
        //     RF_TSel<= #2 4'b0000;//making sure RF_TSel deactiveted for unintended writings

        // end
    end

endmodule
