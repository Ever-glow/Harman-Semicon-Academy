`timescale 1ns / 1ps

module tb_RV32I ();

    logic clk;
    logic reset;
    logic [7:0] GPOA;
    logic [7:0] GPIB;
    wire [7:0] GPIOC;
    wire [7:0] GPIOD;
    logic [3:0] fndComm;
    logic [7:0] fndFont;

    MCU dut (.*);

    always #5 clk = ~clk;

    initial begin
        clk = 0; reset = 1;
        #10 reset = 0;
    end
endmodule
