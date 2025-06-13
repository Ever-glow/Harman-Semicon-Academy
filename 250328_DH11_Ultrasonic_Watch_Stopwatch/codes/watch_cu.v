`timescale 1ns / 1ps

module watch_cu(
    input clk,
    input reset,
    input i_btn_sec,
    input i_btn_min,
    input i_btn_hour,
    output reg o_sec,
    output reg o_min,
    output reg o_hour
);

    parameter IDLE = 2'b00, SEC = 2'b01, MIN = 2'b10, HOUR = 2'b11;

    reg [1:0] state, next;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state <= IDLE;  // 초기 상태 지정
        end else begin
            state <= next;
        end
    end

    always @(*) begin
        next = state;
        case (state)
            IDLE: next = i_btn_sec ? SEC :
                         i_btn_min ? MIN :
                         i_btn_hour ? HOUR : IDLE;
            SEC : next = i_btn_sec ? SEC : IDLE;
            MIN : next = i_btn_min ? MIN : IDLE;
            HOUR: next = i_btn_hour ? HOUR: IDLE;
            default: next = IDLE;
        endcase
    end

    always @(*) begin
        o_sec  = 1'b0;
        o_min  = 1'b0;
        o_hour = 1'b0;
        case (state)
            IDLE: begin
                o_sec  = 1'b0;
                o_min  = 1'b0;
                o_hour = 1'b0;
            end 
            SEC: begin
                o_sec  = 1'b1;
                o_min  = 1'b0;
                o_hour = 1'b0;
            end
            MIN: begin
                o_sec  = 1'b0;
                o_min  = 1'b1;
                o_hour = 1'b0;
            end
            HOUR: begin
                o_sec  = 1'b0;
                o_min  = 1'b0;
                o_hour = 1'b1;
            end
            default: begin
                o_sec  = 1'b0;
                o_min  = 1'b0;
                o_hour = 1'b0;
            end
        endcase
    end

endmodule
