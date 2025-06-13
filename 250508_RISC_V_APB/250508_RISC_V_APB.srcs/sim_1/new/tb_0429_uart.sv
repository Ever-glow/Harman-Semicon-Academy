`timescale 1ns / 1ps
module tb_0429_uart;
  
    // 1) 클럭 & 리셋
    logic clk = 0;
    always #5 clk = ~clk;         // 100 MHz
    logic reset = 1;
    initial begin
        #20 reset = 0;
    end

    // 2) APB 인터페이스 신호
    logic        PSEL, PENABLE, PWRITE;
    logic [3:0]  PADDR;
    logic [31:0] PWDATA;
    logic [31:0] PRDATA;
    logic        PREADY;

    // 3) UART 물리선
    logic rx, tx;

    // 4) DUT 인스턴스
    uart_fifo dut_uart (
        .PCLK    (clk),
        .PRESET  (reset),
        .PADDR   (PADDR),
        .PWDATA  (PWDATA),
        .PWRITE  (PWRITE),
        .PENABLE (PENABLE),
        .PSEL    (PSEL),
        .PRDATA  (PRDATA),
        .PREADY  (PREADY),
        .rx      (rx),
        .tx      (tx)
    );


    


endmodule

