`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.05.2024 16:37:57
// Design Name: 
// Module Name: multiplier_compresssorConventional16_tb
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


module multiplier_compresssorConventional16_tb();
logic[15:0]a,b;
logic[33:0]P;
logic[100:0] errors;


multiplier_compresssorConventional16 dut(a,b,P);
initial begin
    errors = 0;
  end

  initial begin

    for (int i = 65530; i <= 65535; i++) begin
      for (int j = 0; j < 65535; j++) begin
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
