`timescale 1ns / 1ps

module FND_Periph (
    // global signal
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 3:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    // outport signals
    output logic [3:0] fndComm,
    output logic [7:0] fndFont
);
    logic        FCR;
    logic [13:0] FDR;
    logic [ 3:0] FPR;

    APB_SlaveIntf_FND U_APB_Intf (.*);
    FND_controller U_FND_IP (.*);
endmodule

module APB_SlaveIntf_FND (
    // global signal
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 3:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    // internal signals
    output logic        FCR,
    output logic [13:0] FDR,
    output logic [ 3:0] FPR
);
    logic [31:0] slv_reg0, slv_reg1, slv_reg2; //, slv_reg3;

    assign FCR = slv_reg0[0];
    assign FDR = slv_reg1[13:0];
    assign FPR = slv_reg2[3:0];

    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            slv_reg0 <= 0;
            slv_reg1 <= 0;
            slv_reg2 <= 0;
            // slv_reg3 <= 0;
        end else begin
            if (PSEL && PENABLE) begin
                PREADY <= 1'b1;
                if (PWRITE) begin
                    case (PADDR[3:2])
                        2'd0: slv_reg0 <= PWDATA;
                        2'd1: slv_reg1 <= PWDATA;
                        2'd2: slv_reg2 <= PWDATA;
                        // 2'd3: slv_reg3 <= PWDATA;
                    endcase
                end else begin
                    PRDATA <= 32'bx;
                    case (PADDR[3:2])
                        2'd0: PRDATA <= slv_reg0;
                        2'd1: PRDATA <= slv_reg1;
                        2'd2: PRDATA <= slv_reg2;
                        // 2'd3: PRDATA <= slv_reg3;
                    endcase
                end
            end else begin
                PREADY <= 1'b0;
            end
        end
    end

endmodule
/*
module FND (
    input [13:0] bcd,
    input PCLK,
    output logic [3:0] fndComm,
    output logic [7:0] fndFont
);

    assign fndComm = (FCR) ? 4'b0000 : 4'b1111; 

    always_comb begin
        case(FDR)
            4'h0: fndFont = 8'hc0; 
            4'h1: fndFont = 8'hf9; 
            4'h2: fndFont = 8'ha4; 
            4'h3: fndFont = 8'hb0; 
            4'h4: fndFont = 8'h99; 
            4'h5: fndFont = 8'h92; 
            4'h6: fndFont = 8'h82; 
            4'h7: fndFont = 8'hf8; 
            4'h8: fndFont = 8'h80; 
            4'h9: fndFont = 8'h90; 
            4'ha: fndFont = 8'h88; 
            4'hb: fndFont = 8'h83; 
            4'hc: fndFont = 8'hc6; 
            4'hd: fndFont = 8'ha1; 
            4'he: fndFont = 8'h86;  
            4'hf: fndFont = 8'h8e; 
            default: fndFont = 8'hff;
        endcase
    end

endmodule*/

module FND_controller(
    input [13:0] FDR,
    input FCR,
    input [ 3:0] FPR,
    input PCLK, PRESET,
    output logic [3:0] fndComm,
    output logic [7:0] fndFont
    );

    logic dot;
    logic [7:0] segfont;
    logic [3:0] segcomm;
    wire [3:0] w_bcd, w_digit_1, w_digit_10, w_digit_100, w_digit_1000;
    wire [1:0] w_seg_sel;
    wire o_clk;

    assign fndComm = (FCR) ? segcomm : 4'b1111;
    assign fndFont = {dot, segfont[6:0]};

    bcd2seg u_bcd2seg(
        .bcd(w_bcd),
        .seg(segfont)
    );

    two2four u_two2four(
        .sel(w_seg_sel),
        .seg_comm(segcomm)
    );

    digit_splitter u_digit_splitter(
        .bcd(FDR),
        .digit_1(w_digit_1),
        .digit_10(w_digit_10),
        .digit_100(w_digit_100),
        .digit_1000(w_digit_1000)
    );

    mux_4x1 u_mux_4x1(
        .sel(w_seg_sel),
        .digit_1(w_digit_1),
        .digit_10(w_digit_10),
        .digit_100(w_digit_100),
        .digit_1000(w_digit_1000),
        .bcd(w_bcd)
    );

    counter_4 u_counter_4(
        .clk(o_clk),
        .reset(PRESET),
        .o_sel(w_seg_sel)
    );

    clk_divider u_clk_divider(
        .clk(PCLK),
        .reset(PRESET),
        .o_clk(o_clk)
    );

    dotgen u_dotgen(
        .sel(w_seg_sel),
        .x(FPR),
        .y(dot)
    );

endmodule

module clk_divider (
    input clk, reset,
    output o_clk
);

    parameter COUNT = 500_000;
    reg [$clog2(COUNT)-1:0] r_counter;
    reg r_clk;
    assign o_clk = r_clk;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_counter <= 0;
            r_clk <= 1'b0;
        end
        else begin
            if (r_counter == COUNT - 1) begin //100Mhz -> 100hz
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

module counter_4 (
    input clk,
    input reset,
    output [1:0] o_sel
);

    reg [1:0] r_counter;
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

module dotgen(
    input [1:0] sel,
    input [3:0] x,
    output reg y
);
    always @(*) begin
        y = 1'b1;
        case (sel)
            2'b00: y = x[0];
            2'b01: y = x[1];
            2'b10: y = x[2];
            2'b11: y = x[3];
        endcase
    end
endmodule

module two2four (
    input [1:0] sel,
    output reg [3:0] seg_comm
);

    always @(*) begin
        case(sel) 
            2'b00: begin 
                seg_comm = 4'b1110; 
                end
            2'b01: begin 
                seg_comm = 4'b1101; 
                end
            2'b10: begin 
                seg_comm = 4'b1011;
                end
            2'b11: begin 
                seg_comm = 4'b0111;
                end
            default: seg_comm = 4'b1110;
        endcase
    end
    
endmodule

module digit_splitter (
    input [13:0] bcd,
    output [3:0] digit_1,
    output [3:0] digit_10,
    output [3:0] digit_100,
    output [3:0] digit_1000
);
    
    assign digit_1     = bcd % 10;
    assign digit_10    = bcd / 10   % 10; 
    assign digit_100   = bcd / 100  % 10;
    assign digit_1000  = bcd / 1000 % 10;

endmodule

module mux_4x1 (
    input [1:0] sel,
    input [3:0] digit_1, digit_10, digit_100, digit_1000,
    output [3:0] bcd
);
    reg [3:0] r_bcd;
    assign bcd = r_bcd;

    always @(*) begin
        case (sel)
            2'b00: r_bcd = digit_1;
            2'b01: r_bcd = digit_10;
            2'b10: r_bcd = digit_100;
            2'b11: r_bcd = digit_1000; 
            default: r_bcd = digit_1;
        endcase
    end
    
endmodule

module bcd2seg (
    input [3:0] bcd,
    output reg [7:0] seg
);
    always @(bcd) begin
        case(bcd)
            4'h0: seg = 8'hc0; 
            4'h1: seg = 8'hf9; 
            4'h2: seg = 8'ha4; 
            4'h3: seg = 8'hb0; 
            4'h4: seg = 8'h99; 
            4'h5: seg = 8'h92; 
            4'h6: seg = 8'h82; 
            4'h7: seg = 8'hf8; 
            4'h8: seg = 8'h80; 
            4'h9: seg = 8'h90; 
            4'ha: seg = 8'h88; 
            4'hb: seg = 8'h83; 
            4'hc: seg = 8'hc6; 
            4'hd: seg = 8'ha1; 
            4'he: seg = 8'h86;  
            4'hf: seg = 8'h8e; 
            default: seg = 8'hff;
        endcase
    end

endmodule