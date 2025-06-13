`timescale 1ns / 1ps

module fnd_controller_time(
    input [6:0] msec,
    input [5:0] sec,
    input [5:0] min,
    input [4:0] hour,
    input clk, reset,
    input sw_mode,
    output [7:0] seg,
    output [3:0] seg_comm
    );

    wire [3:0] w_bcd, w_msec_digit_1, w_msec_digit_10, 
                      w_sec_digit_1 , w_sec_digit_10 ,
                      w_min_digit_1 , w_min_digit_10 ,
                      w_hour_digit_1, w_hour_digit_10;
    wire [2:0] w_seg_sel;
    wire o_clk;
    wire [3:0] w_min_hour, w_msec_sec;
    wire [3:0] w_seg_comm;
    wire [3:0] w_dot;
    
    assign seg_comm = w_seg_comm;

    bcd2seg_t u_bcd2seg(
        .bcd(w_bcd),
        .seg(seg)
    );

    three2eight_t u_three2eight(
        .sel(w_seg_sel),
        .seg_comm(w_seg_comm)
    );

    digit_splitter_t #(.BIT_WIDTH(7)) u_digit_splitter_msec(
        .bcd(msec),
        .digit_1(w_msec_digit_1),
        .digit_10(w_msec_digit_10)
    );

    digit_splitter_t #(.BIT_WIDTH(6)) u_digit_splitter_sec(
        .bcd(sec),
        .digit_1(w_sec_digit_1),
        .digit_10(w_sec_digit_10)
    );

    digit_splitter_t #(.BIT_WIDTH(6)) u_digit_splitter_min(
        .bcd(min),
        .digit_1(w_min_digit_1),
        .digit_10(w_min_digit_10)
    );

    digit_splitter_t #(.BIT_WIDTH(5)) u_digit_splitter_hour(
        .bcd(hour),
        .digit_1(w_hour_digit_1),
        .digit_10(w_hour_digit_10)
    );

    mux_8x1_t u_mux_8x1_min_hour(
        .sel(w_seg_sel),
        .x0(w_min_digit_1),
        .x1(w_min_digit_10),
        .x2(w_hour_digit_1),
        .x3(w_hour_digit_10),
        .x4(4'hf),
        .x5(4'hf),
        .x6(w_dot),
        .x7(4'hf),
        .y(w_min_hour)
    );

    mux_8x1_t u_mux_8x1_msec_sec(
        .sel(w_seg_sel),
        .x0(w_msec_digit_1),
        .x1(w_msec_digit_10),
        .x2(w_sec_digit_1),
        .x3(w_sec_digit_10),
        .x4(4'hf),
        .x5(4'hf),
        .x6(w_dot),
        .x7(4'hf),
        .y(w_msec_sec)
    );

    mux_2x1_t u_mux_2x1(
        .sw_mode(sw_mode),
        .msec_sec(w_msec_sec),
        .min_hour(w_min_hour),
        .display(w_bcd)
    );

    counter_8_t u_counter_8(
        .clk(o_clk),
        .reset(reset),
        .o_sel(w_seg_sel)
    );

    clk_divider_t u_clk_divider(
        .clk(clk),
        .reset(reset),
        .o_clk(o_clk)
    );

    comparator_msec_t u_compare_dot(
        .msec(msec),
        .dot(w_dot)
    );

endmodule

module comparator_msec_t (
    input [6:0] msec,
    output [3:0] dot
);
    assign dot = (msec < 50) ? 4'he: 4'hf;

endmodule

module clk_divider_t (
    input clk, reset,
    output o_clk
);

    parameter COUNT = 100_000;
    reg [$clog2(COUNT)-1:0] r_counter;
    reg r_clk;
    assign o_clk = r_clk;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_counter <= 0;
            r_clk <= 1'b0;
        end
        else begin
            if (r_counter == COUNT - 1) begin //100Mhz -> ?hz
                r_counter <= 0;
                r_clk <= 1'b1;
            end
            else begin
                r_counter <= r_counter + 1;
                r_clk <= 1'b0;
            end
        end
    end

endmodule

module counter_8_t (
    input clk,
    input reset,
    output [2:0] o_sel
);

    reg [2:0] r_counter;
    assign o_sel = r_counter;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_counter <= 0;
        end
        else begin
            r_counter <= r_counter + 1;
        end
    end

    
endmodule

module three2eight_t (
    input [2:0] sel,
    output reg [3:0] seg_comm
);

    always @(sel) begin
        case(sel) 
            3'b000: seg_comm = 4'b1110; 
            3'b001: seg_comm = 4'b1101; 
            3'b010: seg_comm = 4'b1011;
            3'b011: seg_comm = 4'b0111;
            3'b100: seg_comm = 4'b1110;
            3'b101: seg_comm = 4'b1101;
            3'b110: seg_comm = 4'b1011;
            3'b111: seg_comm = 4'b0111;
            default: seg_comm = 4'b1111;
        endcase
    end
    
endmodule

module digit_splitter_t #(parameter BIT_WIDTH = 7) (
    input [BIT_WIDTH-1:0] bcd,
    output [3:0] digit_1,
    output [3:0] digit_10
);
    
    assign digit_1     = bcd % 10;
    assign digit_10    = bcd / 10   % 10; 

endmodule

module mux_8x1_t (
    input      [2:0] sel,
    input      [3:0] x0,
    input      [3:0] x1,
    input      [3:0] x2,
    input      [3:0] x3,
    input      [3:0] x4,
    input      [3:0] x5,
    input      [3:0] x6,
    input      [3:0] x7,
    output reg [3:0] y
);
    always @(*) begin
        case (sel)
            3'b000: y = x0;
            3'b001: y = x1;
            3'b010: y = x2;
            3'b011: y = x3;
            3'b100: y = x4;
            3'b101: y = x5;
            3'b110: y = x6;
            3'b111: y = x7; 
            default: y = 4'hf;
        endcase
    end
endmodule

module mux_2x1_t (
    input sw_mode,
    input  [3:0] msec_sec,
    input  [3:0] min_hour,
    output reg [3:0] display
);
    always @(*) begin
        if (sw_mode == 1'b1) begin
            display = min_hour;
        end
        else if(sw_mode == 1'b0) begin
            display = msec_sec;
        end
    end
 
endmodule

/*
module mux_4x1 (
    input [1:0] sel,
    input [3:0] digit_1_1, digit_1_10, digit_2_1, digit_2_10,
    output [3:0] bcd
);
    reg [3:0] r_bcd;
    assign bcd = r_bcd;

    always @(*) begin
        case (sel)
            2'b00: r_bcd = digit_1_1;
            2'b01: r_bcd = digit_1_10;
            2'b10: r_bcd = digit_2_1;
            2'b11: r_bcd = digit_2_10; 
            default: r_bcd = 4'bx;
        endcase
    end
    
endmodule
*/
module bcd2seg_t (
    input [3:0] bcd,     // [3:0] sum 값
    output reg [7:0] seg
);
    // always 구문 출력으로 reg type을 가져야 한다
    always @(bcd) begin
        case (bcd)
            4'h0: seg = 8'hC0;
            4'h1: seg = 8'hF9;
            4'h2: seg = 8'hA4;
            4'h3: seg = 8'hB0;
            4'h4: seg = 8'h99;
            4'h5: seg = 8'h92;
            4'h6: seg = 8'h82;
            4'h7: seg = 8'hF8;
            4'h8: seg = 8'h80;
            4'h9: seg = 8'h90;
            4'hA: seg = 8'h88;
            4'hB: seg = 8'h83;
            4'hC: seg = 8'hC6;
            4'hD: seg = 8'hA1;
            4'hE: seg = 8'h7f;
            4'hF: seg = 8'hff; // segment off
            default: seg = 8'hff;
        endcase
    end
endmodule