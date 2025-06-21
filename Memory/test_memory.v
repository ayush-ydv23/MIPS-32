`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/29/2025 02:25:59 PM
// Design Name: 
// Module Name: test_memory
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


module test_memory();


reg [8:0] a,dpra;
reg clk,we;
reg [31:0] d;
wire [31:0] dpo;

initial begin
clk<=0;
forever #10 clk<=~clk;
end


initial begin
we<=0;
#100;
a=9'd1;
we=1;
dpra=1'd0;
d=32'ha0;
#20
a=9'd2;
we=1;
dpra=1'd0;
d=32'hb0;
#20
a=9'd3;
we=1;
dpra=1'd0;
d=32'hc0;
#20
a=9'd4;
we=1;
dpra=1'd0;
d=32'hd0;
#20
a=9'd4;
we=0;
dpra=1'd1;
d=32'hd0;
#20
a=9'd4;
we=0;
dpra=9'd2;
d=32'hd0;
#20
a=9'd4;
we=0;
dpra=9'd3;
d=32'hd0;
#20
a=9'd4;
we=0;
dpra=9'd4;
d=32'hd0;
end

memory_wrapper mem(a,d,dpra,clk,we,dpo);
endmodule
