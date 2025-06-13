`timescale 1ns / 1ps

module watch1_dp(
    input clk, reset,
    input btn_sec, btn_min, btn_hour,
    output [6:0] o_msec,
    output [5:0] o_sec,
    output [5:0] o_min,
    output [4:0] o_hour
    );

    wire w_clk_100hz;
    wire w_msec_tick, w_sec_tick, w_min_tick;


    clk_div_100Hz_w u_clk_div(
        .clk(clk),
        .reset(reset),
        .o_clk(w_clk_100hz)
    );

    time_counter_w #(
        .TICK_COUNT(100), 
        .BIT_WIDTH(7)) 
        u_time_msec(
        .clk(clk),
        .reset(reset),
        .tick(w_clk_100hz),
        .btn(btn_sec),
        .o_time(o_msec),
        .o_tick(w_msec_tick)
    );

    time_counter_w #(
        .TICK_COUNT(60), 
        .BIT_WIDTH(6)) 
        u_time_sec(
        .clk(clk),
        .reset(reset),
        .tick(w_msec_tick),
        .btn(btn_min),
        .o_time(o_sec),
        .o_tick(w_sec_tick)
    );

    time_counter_w #(
        .TICK_COUNT(60), 
        .BIT_WIDTH(6)) 
        u_time_min(
        .clk(clk),
        .reset(reset),
        .tick(w_sec_tick),
        .btn(btn_hour),
        .o_time(o_min),
        .o_tick(w_min_tick)
    );

    time_counter_hour u_time_hour(
        .clk(clk),
        .reset(reset),
        .tick(w_min_tick),
        .btn(),
        .o_time(o_hour),
        .o_tick()
    );

endmodule

module time_counter_w #(parameter TICK_COUNT = 100, BIT_WIDTH = 7) (
    input clk,
    input reset,
    input btn,
    input tick,
    output [BIT_WIDTH-1:0] o_time,
    output o_tick
);
    
    reg [$clog2(TICK_COUNT)-1:0] count_reg, count_next;
    reg tick_reg, tick_next;

    assign o_time = count_reg;
    assign o_tick = tick_reg;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            count_reg <= 0;
            tick_reg <= 0;
        end
        else begin
            tick_reg <= tick_next;
            count_reg <= count_next;
        end
    end
    

    always @(*) begin
        count_next = count_reg;
        tick_next = 1'b0;
        if (tick == 1'b1) begin
            if (count_reg == TICK_COUNT-1) begin
                count_next = 0;
                tick_next = 1'b1;
            end
            else begin
                count_next = count_reg + 1;
                tick_next = 1'b0;
            end
        end
        if (btn == 1'b1) begin
            tick_next = tick_next + 1;
        end
    end

endmodule

module time_counter_hour #(parameter TICK_COUNT = 24, BIT_WIDTH = 5) (
    input clk,
    input reset,
    input btn,
    input tick,
    output [BIT_WIDTH-1:0] o_time,
    output o_tick
);
    
    reg [$clog2(TICK_COUNT)-1:0] count_reg, count_next;
    reg tick_reg, tick_next;

    assign o_time = count_reg;
    assign o_tick = tick_reg;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            count_reg <= 4'b1100;
            tick_reg <= 0;
        end
        else begin
            tick_reg <= tick_next;
            count_reg <= count_next;
        end
    end
    

    always @(*) begin
        count_next = count_reg;
        tick_next = 1'b0;
        if (tick == 1'b1) begin
            if (count_reg == TICK_COUNT-1) begin
                count_next = 0;
                tick_next = 1'b1;
            end
            else begin
                count_next = count_reg + 1;
                tick_next = 1'b0;
            end
        end
        if (btn == 1'b1) begin
            tick_next = tick_next + 1;
        end
    end

endmodule

module clk_div_100Hz_w(
    input clk,
    input reset,
    output o_clk
);
    
    parameter FCOUNT = 1_000_000;
    reg [$clog2(FCOUNT)-1:0] count_reg, count_next;
    reg clk_reg, clk_next;  // 출력을 f/f으로 내보내기 위해서

    assign o_clk = clk_reg; // 최종 출력

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            count_reg <= 0;
            clk_reg <= 0;
        end else  begin
            count_reg <= count_next;
            clk_reg <= clk_next;
        end
    end

    always @(*) begin
        if (count_reg == FCOUNT - 1) begin
            count_next = 0;
            clk_next = 1'b1;
        end else begin
            count_next = count_reg + 1;
            clk_next = 1'b0;
        end
    end

endmodule
