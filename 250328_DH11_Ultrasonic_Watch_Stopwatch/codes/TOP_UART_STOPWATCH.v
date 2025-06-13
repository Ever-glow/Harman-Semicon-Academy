`timescale 1ns / 1ps

module TOP_UART_STOPWATCH(
    // stopwatch I/O
    input clk,
    input reset,
    input btnr,
    input btnl,
    input btnu,
    input btnd,
    input [1:0] sw_display,
    input sw_convert,
    input [7:0] data,
    input done,
    output [3:0] fnd_comm,
    output [7:0] fnd_font,
    output [3:0] led,

    // print_gen으로 전송할 데이터터
    output [5:0] data_sec, 
    output [5:0] data_min, 
    output [4:0] data_hour,
    
    // uart fifo I/O
    input rx,
    output tx
    );

    wire [4:0] w_control;

    top_uart2stopwatch U_UART2STOPWATCH(
        .clk(clk),
        .reset(reset),
        .data(data),
        .data_valid(done),
        .control(w_control)
    );

    top_stopwatch U_STOPWATCH(
        .clk(clk),
        .btnc(reset),
        .btnr(btnr),
        .btnl(btnl),
        .btnu(btnu),
        .btnd(btnd),
        .sw_display(sw_display),
        .sw_convert(sw_convert),
        .fnd_comm(fnd_comm),
        .fnd_font(fnd_font),
        .led(led),
        .control(w_control),
        .data_sec(data_sec),
        .data_min(data_min),
        .data_hour(data_hour)
    );

endmodule
