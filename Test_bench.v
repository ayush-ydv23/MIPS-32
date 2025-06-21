`timescale 1ns / 1ps

module RegisterFile_tb;

    // Inputs
    reg [4:0] rd, rs, rt;
    reg clk;
    reg we;
    reg [31:0] write_data;
    reg rst;

    // Outputs
    wire [31:0] rs_out, rt_out;

    // Instantiate the Unit Under Test (UUT)
    RegisterFile uut (
        .rd(rd), 
        .rs(rs), 
        .rt(rt), 
        .clk(clk), 
        .we(we), 
        .write_data(write_data), 
        .rs_out(rs_out), 
        .rt_out(rt_out), 
        .rst(rst)
    );

    // Clock generation
    always #5 clk = ~clk; // 10ns clock period

    initial begin
        // Initial values
        clk = 0;
        rst = 1;
        we = 0;
        rd = 0;
        rs = 0;
        rt = 0;
        write_data = 0;

        // Reset pulse
        #10;
        rst = 0;

        // Write 32'hDEADBEEF to register 5
        rd = 5;
        write_data = 32'hDEADBEEF;
        we = 1;
        #10; // Ensure we catch a negative edge for write

        // Disable write
        we = 0;
        #10;

        // Set rs to 5 to read from it
        rs = 5;
        #10; // Wait for posedge read

        $display("Value in register 5 (rs_out): %h", rs_out);

        // Read from a register that hasn't been written
        rs = 10;
        #10;
      $display("Value in register 10 (rs_out): %h", rs_out);

        $finish;
    end

endmodule
