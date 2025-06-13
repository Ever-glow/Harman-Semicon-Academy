`timescale 1ns / 1ps

module top_dist(
    input clk, reset,
    input start,
    input echo,
    input rx,
    input [7:0] start_define,
    output tx,
    output trig,
    output done,
    output [7:0] seg,
    output [3:0] seg_comm,
    output [8:0] dist
    );

    wire [8:0] w_distance, w_bcd;
    wire w_start, w_done;
    reg [8:0] r_bcd;

    reg  [7:0] r_out_data;
    reg start_uart;

    always @(posedge clk, posedge reset) begin
        r_out_data <= start_define;
        if (reset) begin
            r_out_data <= 8'b0;
            start_uart <= 1'b0;
        end
        else if (r_out_data == 8'h53 || r_out_data == 8'h73) // 'S' or 's'
            start_uart <= 1'b1;
        else
            start_uart <= 1'b0;
    end
    
    assign w_start = start_uart || start;

    assign w_bcd = r_bcd;
    assign done = w_done;
    assign dist = w_bcd;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_bcd <= 0;
        end
        else if (w_done) begin
            r_bcd <= w_distance;
        end
    end

    top_distance U_TOP_DISTANCE(
        .clk(clk),
        .reset(reset),
        .start(w_start),
        .echo(echo),
        .trig(trig),
        .distance(w_distance),
        .done(w_done)
    );

    fnd_controller_dist U_FND_CON(
        .bcd(w_bcd),
        .clk(clk),
        .reset(reset),
        .seg(seg),
        .seg_comm(seg_comm)
    );

endmodule
