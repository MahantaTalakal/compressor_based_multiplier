`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.03.2024 16:49:51
// Design Name: 
// Module Name: multiplier_compressor_better
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module multiplier_compressor_better(
input logic[7:0]a,b,
output logic[16:0]P
    );
    
logic[63:0] p;
logic[40:1] s,ca;
logic [25:1]cb;
logic[15:0]a1,b1;
    
   and(p[0],a[0],b[0]);
   and(p[1],a[1],b[0]);
   and(p[2],a[2],b[0]);
   and(p[3],a[3],b[0]);
   and(p[4],a[4],b[0]);
   and(p[5],a[5],b[0]);
   and(p[6],a[6],b[0]);
   and(p[7],a[7],b[0]);
   
   //layer 2
   and(p[8],a[0],b[1]);
   and(p[9],a[1],b[1]);
   and(p[10],a[2],b[1]);
   and(p[11],a[3],b[1]);
   and(p[12],a[4],b[1]);
   and(p[13],a[5],b[1]);
   and(p[14],a[6],b[1]);
   and(p[15],a[7],b[1]);
   
   //layer 3
   and(p[16],a[0],b[2]);
   and(p[17],a[1],b[2]);
   and(p[18],a[2],b[2]);
   and(p[19],a[3],b[2]);
   and(p[20],a[4],b[2]);
   and(p[21],a[5],b[2]);
   and(p[22],a[6],b[2]);
   and(p[23],a[7],b[2]);
   
   //layer 4
   and(p[24],a[0],b[3]);
   and(p[25],a[1],b[3]);
   and(p[26],a[2],b[3]);
   and(p[27],a[3],b[3]);
   and(p[28],a[4],b[3]);
   and(p[29],a[5],b[3]);
   and(p[30],a[6],b[3]);
   and(p[31],a[7],b[3]);
   
   //layer 5
   and(p[32],a[0],b[4]);
   and(p[33],a[1],b[4]);
   and(p[34],a[2],b[4]);
   and(p[35],a[3],b[4]);
   and(p[36],a[4],b[4]);
   and(p[37],a[5],b[4]);
   and(p[38],a[6],b[4]);
   and(p[39],a[7],b[4]);
   
   //layer 6
   and(p[40],a[0],b[5]);
   and(p[41],a[1],b[5]);
   and(p[42],a[2],b[5]);
   and(p[43],a[3],b[5]);
   and(p[44],a[4],b[5]);
   and(p[45],a[5],b[5]);
   and(p[46],a[6],b[5]);
   and(p[47],a[7],b[5]);
   
   //layer 7
   and(p[48],a[0],b[6]);
   and(p[49],a[1],b[6]);
   and(p[50],a[2],b[6]);
   and(p[51],a[3],b[6]);
   and(p[52],a[4],b[6]);
   and(p[53],a[5],b[6]);
   and(p[54],a[6],b[6]);
   and(p[55],a[7],b[6]);
   
   //layer 8
   and(p[56],a[0],b[7]);
   and(p[57],a[1],b[7]);
   and(p[58],a[2],b[7]);
   and(p[59],a[3],b[7]);
   and(p[60],a[4],b[7]);
   and(p[61],a[5],b[7]);
   and(p[62],a[6],b[7]);
   and(p[63],a[7],b[7]);

Half_adder s1(p[2],p[9],s[1],ca[1]);

exact_compressor s2(1'b0,p[3],p[10],p[17],p[24],s[2],ca[2],cb[2]);

exact_compressor s3(p[4],p[11],p[18],p[25],p[32],s[3],ca[3],cb[3]);

exact_compressor s4(p[5],p[12],p[19],p[26],p[33],s[4],ca[4],cb[4]);

exact_compressor s5(p[6],p[13],p[20],p[27],p[34],s[5],ca[5],cb[5]);

exact_compressor s6(p[7],p[14],p[21],p[28],p[35],s[6],ca[6],cb[6]);

exact_compressor s7(p[15],p[22],p[29],p[36],p[43],s[7],ca[7],cb[7]);

exact_compressor s8(p[23],p[30],p[37],p[44],p[51],s[8],ca[8],cb[8]);

exact_compressor s9(p[31],p[38],p[45],p[52],p[59],s[9],ca[9],cb[9]);

exact_compressor s10(p[39],p[46],p[53],p[60],1'b0,s[10],ca[10],cb[10]);

Full_adder s11(p[47],p[54],p[61],s[11],ca[11]);

Half_adder s12(p[55],p[62],s[12],ca[12]);

Half_adder s13(p[41],p[48],s[13],ca[13]);

Full_adder s14(p[42],p[49],p[56],s[14],ca[14]);

Half_adder s15(p[50],p[57],s[15],ca[15]);

Half_adder s16(s[3],ca[2],s[16],ca[16]);

exact_compressor s17(s[4],ca[3],cb[3],p[40],1'b0,s[17],ca[17],cb[17]);

exact_compressor s18(s[5],ca[4],cb[4],s[13],1'b0,s[18],ca[18],cb[18]);

exact_compressor s19(s[6],ca[5],cb[5],s[14],ca[13],s[19],ca[19],cb[19]);

exact_compressor s20(s[7],ca[6],cb[6],s[15],ca[14],s[20],ca[20],cb[20]);

exact_compressor s21(s[8],ca[7],cb[7],p[58],ca[15],s[21],ca[21],cb[21]);

Full_adder s22(s[9],ca[8],cb[8],s[22],ca[22]);

Full_adder s23(s[10],ca[9],cb[9],s[23],ca[23]);

Full_adder s24(s[11],ca[10],cb[10],s[24],ca[24]);

Half_adder s25(s[12],ca[11],s[25],ca[25]);

Half_adder s26(p[63],ca[12],s[26],ca[26]);



Half_adder s27(s[18],ca[17],s[27],ca[27]);

Full_adder s28(s[19],ca[18],cb[18],s[28],ca[28]);

Full_adder s29(s[20],ca[19],cb[19],s[29],ca[29]);

Full_adder s30(s[21],ca[20],cb[20],s[30],ca[30]);

Full_adder s31(s[22],ca[21],cb[21],s[31],ca[31]);

Half_adder s32(s[23],ca[22],s[32],ca[32]);

Half_adder s33(s[24],ca[23],s[33],ca[33]);

Half_adder s34(s[25],ca[24],s[34],ca[34]);

Half_adder s35(s[26],ca[25],s[35],ca[35]);

assign a1={ca[26],s[35],s[34],s[33],s[32],s[31],s[30],s[29],s[28],s[27],s[17],s[16],s[2],s[1],p[1],p[0]};
assign b1={ca[35],ca[34],ca[33],ca[32],ca[31],ca[30],ca[29],ca[28],ca[27],cb[17],ca[16],cb[2],ca[1],p[16],p[8],1'b0};

ripple_16 finalsum(a1,b1,0,P);

endmodule

