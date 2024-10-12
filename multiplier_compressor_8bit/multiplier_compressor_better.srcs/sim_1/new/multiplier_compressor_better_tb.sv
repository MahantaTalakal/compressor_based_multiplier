`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.03.2024 17:19:32
// Design Name: 
// Module Name: multiplier_compressor_better_tb
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


module multiplier_compressor_better_tb();
logic[7:0]a,b;
logic[16:0]P;
logic[100:0] errors;


multiplier_compressor_better dut(a,b,P);
initial begin
    errors = 0;
  end

  initial begin
    for (int i = 1; i < 256; i++) begin
      for (int j = 0; j < 256; j++) begin
        a = i;
        b = j;
        #0.001;
        assert (P == i * j) 
            else begin 
                errors=errors+1;
                $error("%d*%d is %d but got %d....error number:%d", a, b, (i * j), P,errors);
                end
      end
    end
    #1000000;
    $stop;
  end
endmodule

