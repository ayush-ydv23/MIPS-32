`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/29/2025 02:19:29 PM
// Design Name: 
// Module Name: memory_wrapper
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


module memory_wrapper(a,d,dpra,clk,we,dpo);

input [9:0] a,dpra;
input clk,we;
output [31:0] dpo;
input [31:0] d; 
     
    dist_mem_gen_0 your_instance_name (
  .a(a),        // input wire [8 : 0] a
  .d(d),        // input wire [31 : 0] d
  .dpra(dpra),  // input wire [8 : 0] dpra
  .clk(clk),    // input wire clk
  .we(we),      // input wire we
  .dpo(dpo)    // output wire [31 : 0] dpo
);
endmodule

// .a(a),        // input wire [8 : 0] a. This is the write Address
//   .d(d),        // input wire [31 : 0] d. This is the data to be written
//   .dpra(dpra),  // input wire [8 : 0] dpra. This is the read Address
//   .clk(clk),    // input wire clk
//   .we(we),      // input wire we
//   .dpo(dpo)    // output wire [31 : 0] dpo. the value that is read