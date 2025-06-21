// Code your design here
`timescale 1ns / 1ps

module DistributedMemory (
 input wire [9:0] a, input wire[9:0] dpra , input we , input clk ,
 input wire [31 : 0] d , output wire [31 : 0] dpo
);
 reg [31:0] Address_locations [1023:0];
 assign dpo = Address_locations[dpra];
 always @(negedge clk) begin
 if(we) begin
 Address_locations[a] = d;
 end
 else begin
 Address_locations[a] = Address_locations[a];
 end
 end

endmodule
//Reads at positive edge and writes at negative edge
module RegisterFile (
 input wire [4:0] rd , input wire [4:0] rs , input wire [4:0] rt , input clk , input we , input wire [31:0] write_data,
 output [31:0] rs_out , output [31:0] rt_out, input wire rst
);
 reg [31:0] Registers [31:0];
 integer i;
 assign rs_out = Registers[rs];
 assign rt_out = Registers[rt];
 always @(posedge clk or negedge clk ) begin
 if(rst) begin
 for (i = 0; i < 32; i = i + 1) begin
 Registers[i] = 0;
 end
 end
 else begin
 if(we) begin
 Registers[rd] = write_data;
 end
 else begin
 Registers[rd] = Registers[rd];
 end
 end
 end

endmodule
module Splitter (
 input wire [31:0] instruction, output wire [4:0] rt , output wire [4:0] rs , output wire [4:0] rd ,
 output wire [5:0] func , output wire [4:0] shamt , output wire [5:0] opcode , output wire [15:0] address_constant,
 output wire [25:0] jaddress
);
 assign opcode = instruction[31:26];
 assign rd = instruction[25:21];
 assign rt = instruction[20:16];
 assign rs = instruction[15:11];
 assign shamt = instruction[10:6];
 assign func = instruction[5:0];
 assign address_constant = instruction[15:0];
 assign jaddress = instruction[25:0];

endmodule
/*
ALUctrl = 0 => ADD
1 == Sub (inp1 - inp2) == rs - rt
2 == and
3 == xor
4 == or
5 == not
6 == left shift
7 == right shift
8 == set not equal
9 == set equal
10 == set less than
11 == set less than equal
12 == set greater than
13 == set greater than equal
*/
module ALU (
 input wire [31:0] inp1, input wire [31:0] inp2 , input wire [3:0] ALUctrl , input wire clk ,
 output reg [31:0] ALUout , input wire [4:0] shamt
);
 always @(inp1 or inp2 or ALUctrl or shamt ) begin
 case (ALUctrl)
 0: ALUout = inp1 + inp2;
 1: ALUout = inp1 - inp2;
 2: ALUout = inp1 & inp2;
 3: ALUout = inp1 ^ inp2;
 4: ALUout = inp1 | inp2;
 5: ALUout = ~inp1;
 6: ALUout = inp1 << shamt;
 7: ALUout = inp1 >> shamt;
 8: ALUout = ($signed(inp1) != $signed(inp2));
 9: ALUout = ($signed(inp1) == $signed(inp2));
 10: ALUout = ($signed(inp1) < $signed(inp2));
 11: ALUout = ($signed(inp1) <= $signed(inp2));
 12: ALUout = ($signed(inp1) > $signed(inp2));
 13: ALUout = ($signed(inp1) >= $signed(inp2));
 14: ALUout = inp2 << 16;
 default: ALUout = 32'hdeadbeef;
 endcase
 end
endmodule

module PC_Controller (
 input clk , output reg [9:0] PC, input rst , input jump , input [25:0] jaddress , input branch , input [15:0] branchval
);
 always @(posedge clk ) begin
 if(rst) begin
 PC = 0;
 end
 else begin
 if(jump) begin
 PC = jaddress;
 end
 else if(branch) begin
 PC = PC + 1 + branchval;
 end
 else begin
 PC = PC + 1;
 end
 end
 end
endmodule

/*
ALUctrl = 0 => ADD
1 == Sub (inp1 - inp2) == rs - rt
2 == and
3 == xor
4 == or
5 == not
6 == left shift
7 == right shift
8 == set not equal
9 == set equal
10 == set less than
11 == set less than equal
12 == set greater than
13 == set greater than equal
*/
module ALU_controller (
 input wire [5:0] opcode , input wire[5:0] func , output reg [3:0] ALUctrl
);
 always @(opcode or func) begin
 if(opcode == 6'b000111 || opcode == 6'b001000) begin // lw or sw
 ALUctrl = 0; // add
 end
 else if(opcode == 1) begin // addi
 ALUctrl = 0;
 end
 else if(opcode == 2) begin // andi
 ALUctrl = 2;
 end
 else if(opcode == 3) begin // ori
 ALUctrl = 4;
 end
 else if(opcode == 4) begin // xori
 ALUctrl = 3;
 end
 else if(opcode == 5) begin // addui
 ALUctrl = 0;
 end
 else if(opcode == 6'b001001 || opcode == 6'b010100 || opcode == 6'b010110) begin // slti and ble , bleu
 ALUctrl = 10;
 end
 else if(opcode == 6'b001010 || opcode == 6'b010000) begin // seq, beq
 ALUctrl = 9;
 end
 else if(opcode == 6'b010001) begin // bne
 ALUctrl = 8;
 end
 else if(opcode == 6'b010010 || opcode == 6'b010111) begin // bgt ,bgtu
 ALUctrl = 12;
 end
 else if(opcode == 6'b010011) begin // bgte
 ALUctrl = 13;
 end
 else if(opcode == 6'b010101) begin // bleq
 ALUctrl = 11;
 end
 else if(opcode == 6'b001011) begin//lui
 ALUctrl = 14;
 end
 else begin
 if(opcode == 6'b000000) begin
 case (func)
 0: ALUctrl = 0;
 1: ALUctrl = 1;
 2: ALUctrl = 2;
 3: ALUctrl = 4;
 4: ALUctrl = 5;
 5: ALUctrl = 3;
 6: ALUctrl = 0;
 7: ALUctrl = 1;
 8: ALUctrl = 10;
 9: ALUctrl = 6;
 10: ALUctrl = 7;
 default: ALUctrl = 15;
 endcase
 end
 else begin
 ALUctrl = 4'b1111;
 end
 end
 end
endmodule

module SignExtender (
 input wire[15:0] inp , output wire [31:0] out
);
 assign out = {{16{inp[15]}}, inp};
endmodule

module mux2_1 #(
 parameter size = 32
) (
 input wire [size - 1 : 0] inp0 , input wire [size - 1 : 0] inp1, input select , output wire[size - 1 : 0] out
);
 assign out = (select)? inp0 : inp1;
endmodule
module Controller (
 input clk , input wire [5:0] opcode , output reg write_reg , output reg data_write ,
 output reg immediate , output reg jump, output reg branch , output reg jal , output reg select_ALU_or_Mem,//0 for ALU, 1 for MEM
 input rst
);
 always @(opcode or rst) begin
 if(rst) begin
 immediate = 0;
 jump = 0;
 branch = 0;
 select_ALU_or_Mem = 0;
 data_write = 0;
 write_reg = 0;
 jal = 0;
 end
 else if(opcode == 0) begin // Rtype instruction
 data_write = 0;
 write_reg = 1;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 end
 else if(opcode == 1 ||opcode == 2 ||opcode == 3 ||opcode == 4 ||opcode == 5) begin //addi, andi , xori, ori , addiu
 data_write = 0;
 write_reg = 1;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 end
 else if(opcode == 7) begin // lw
 write_reg = 1;
 data_write = 0;
 immediate = 1;
 select_ALU_or_Mem = 1;
 branch = 0;
 jump = 0;
 jal = 0;
 end
 else if(opcode == 8) begin // sw
 write_reg = 0;
 data_write = 1;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 end
 else if(opcode == 9 || opcode == 10) begin // slti , seq
 write_reg = 1;
 data_write = 0;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 end
 else if(opcode == 6'b001011) begin //lui
 write_reg = 1;
 data_write = 0;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 end
 else if(opcode == 16 || opcode == 17 ||opcode == 18 || opcode == 19 || opcode == 20 || opcode == 21 || opcode == 22 || opcode == 23) begin //all branches
 write_reg = 0;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 1;
 jump = 0;
 jal = 0;
 end
 else if(opcode == 6'b011000 || opcode == 6'b011001) begin // All jump except jal
 write_reg = 0;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 1;
 jal = 0;
 end
 else if(opcode == 6'b011010) begin // jal handled seperately as it requires to store the value of PC + 4 into $ra
 write_reg = 1;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 1;
 jal = 1;
 end
 else begin
 write_reg = 0;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 end
 end
endmodule


module CPU (
 input rst , input clk , input wire [31:0] inst_data ,
 input wire [9:0] address , input wire write_instruction , input wire write_data,
 output wire [31:0] OutputOfRs
);
// .a(a), // input wire [8 : 0] a. This is the write Address
// .d(d), // input wire [31 : 0] d. This is the data to be written
// .dpra(dpra), // input wire [8 : 0] dpra. This is the read Address
// .clk(clk), // input wire clk
// .we(we), // input wire we
// .dpo(dpo) // output wire [31 : 0] dpo. the value that is read
 wire [9:0] PC;
 wire [31:0] instruction;
 wire [31:0] ALU_out;
 wire [9:0] memory_in;
 wire [31:0] rt_out , rs_out , memory_write , memory_out , rt_or_address_to_ALU, extended_address;
 wire [4:0] rt , rs , rd , shamt;
 wire [5:0] opcode , func;
 wire [15:0] address_constant;
 wire [25:0] jaddress;
 wire branch , jump , jal , mem_write , immediate, select_ALU_or_Mem, write_reg;
 wire [4:0] wire_going_into_rs;
 //mem_write instead of data_write
 wire [3:0] ALUctrl;
 assign OutputOfRs = rs_out;
 assign wire_going_into_rs = (branch | mem_write)? rd : rs;
 //this thing is to handle jal
 wire[4:0] write_in_Register;
 assign write_in_Register = (jal)? 5'd3 : rd;
 wire [31:0] write_data_in_register , wire_going_into_register_rd;
 assign wire_going_into_register_rd = (jal)? {22'b0 , PC} : write_data_in_register;
 //write_data_in_register is coming from the last mux(selecting ALU or memory Output)
 //wire_going_into_register_rd is write data that is actually going into register file
 //write_in_register goes into rd
 // mux2_1 rt_ALU (rt_out , extended_address , immediate , rt_or_address_to_ALU);
 assign rt_or_address_to_ALU = (immediate)? extended_address : rt_out;
 SignExtender sign_extend_addr_const(.inp(address_constant) , .out(extended_address));
 // mux2_1 Last_mem_stage(ALU_out , memory_out , select_ALU_or_Mem , write_data_in_register);
 assign write_data_in_register = (select_ALU_or_Mem)? memory_out : ALU_out;
 DistributedMemory instruction_mem(.a(address),.d(inst_data),.dpra(PC),.clk(clk),.we(write_instruction),.dpo(instruction));

// mux2_1 #(.size(10)) ALU_Mem_write_address (ALU_out[9:0] , address , write_data , memory_in);
 assign memory_in = (write_data)? address : ALU_out[9:0];
//  mux2_1 What_to_write_decider(rt_out , inst_data , write_data , memory_write);
 assign memory_write = (write_data)? inst_data : rt_out;
 DistributedMemory data_mem(.a(memory_in) , .d(memory_write) , .dpra(ALU_out[9:0]) , .clk(clk) , .we(mem_write | write_data) , .dpo(memory_out));

 Splitter split(.instruction(instruction) , .rt(rt) , .rs(rs) , .rd(rd) , .shamt(shamt), .func(func), .opcode(opcode) , .address_constant(address_constant) , .jaddress(jaddress));
 Controller brain(.clk(clk) , .opcode(opcode) , .write_reg(write_reg) , .data_write(mem_write) ,
 .immediate(immediate) , .jump(jump) , .branch(branch) , .jal(jal) , .select_ALU_or_Mem(select_ALU_or_Mem) ,
 .rst(rst));
 RegisterFile RAM(.rd(write_in_Register) , .rs(wire_going_into_rs) , .rt(rt) , .clk(clk) , .we(write_reg) , .write_data(wire_going_into_register_rd) , .rst(rst) , .rt_out(rt_out) , .rs_out(rs_out));

 ALU_controller nerves(.opcode(opcode) , .func(func) , .ALUctrl(ALUctrl));
 ALU brawn(.inp1(rs_out) , .inp2(rt_or_address_to_ALU) , .ALUctrl(ALUctrl) , .clk(clk) , .ALUout(ALU_out) , .shamt(shamt));
 PC_Controller legs(.clk(clk) , .PC(PC) , .rst(rst) , .jump(jump) , .jaddress(jaddress) , .branch(branch & ALU_out[0]) , .branchval(address_constant));

endmodule
