`timescale 1ns / 1ps

module Timer_Periph (
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
    output logic        PREADY
    // export signals
);

    //logic [ 1:0] TCR;
    logic [31:0] TCNT;
    logic [31:0] PSC;
    logic [31:0] ARR;

    logic en;
    logic clear;

    APB_SlaveIntf_timer U_APB_Intf_timer (.*);
    timer U_timer_IP (.*);
endmodule

module APB_SlaveIntf_timer (
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
    output logic        en,
    output logic        clear,
    //output logic [ 1:0] TCR,
    input  logic [31:0] TCNT,
    output logic [31:0] PSC,
    output logic [31:0] ARR

);
    logic [31:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3;

    assign en       = slv_reg0[0];
    assign clear    = slv_reg0[1];
    assign slv_reg1 = TCNT;
    assign PSC      = slv_reg2;
    assign ARR      = slv_reg3;

    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            slv_reg0 <= 0;
            //slv_reg1 <= 0;
            slv_reg2 <= 0;
            slv_reg3 <= 0;
        end else begin
            if (PSEL && PENABLE) begin
                PREADY <= 1'b1;
                if (PWRITE) begin
                    case (PADDR[3:2])
                        2'd0: slv_reg0 <= PWDATA;
                        //2'd1: slv_reg1 <= PWDATA;
                        2'd2: slv_reg2 <= PWDATA;
                        2'd3: slv_reg3 <= PWDATA;
                    endcase
                end else begin
                    PRDATA <= 32'bx;
                    case (PADDR[3:2])
                        2'd0: PRDATA <= slv_reg0;
                        2'd1: PRDATA <= slv_reg1;
                        2'd2: PRDATA <= slv_reg2;
                        2'd3: PRDATA <= slv_reg3;
                    endcase
                end
            end else begin
                PREADY <= 1'b0;
            end
        end
    end

endmodule

module timer (
    input logic PCLK,
    input logic PRESET,
    input logic en,
    input logic clear,
    input logic [31:0] PSC,
    input logic [31:0] ARR,
    output logic [31:0] TCNT
);
    logic tick;

    prescaler U_prescaler (.*);
    counter U_Counter (.*);
endmodule

module prescaler (
    input  logic        PCLK,
    input  logic        PRESET,
    input  logic        en,
    input  logic        clear,
    input  logic [31:0] PSC,
    output logic        tick
);
    logic [31 : 0] div_counter;

    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            div_counter <= 0;
            tick <= 0;
        end else begin
            if (en) begin
                if (div_counter == PSC) begin
                    div_counter <= 0;
                    tick <= 1'b1;
                end else begin
                    div_counter <= div_counter + 1;
                    tick <= 1'b0;
                end
            end
            if (clear) begin
                div_counter <= 0;
                tick <= 1'b0;
            end
        end
    end
endmodule

module counter (
    input logic PCLK,
    input logic PRESET,
    input logic tick,
    input logic clear,
    input logic [31:0] ARR,
    output logic [31:0] TCNT
);
    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            TCNT <= 0;
        end else begin
            if (tick) begin
                if (TCNT == ARR) begin
                    TCNT <= 0;
                end else begin
                    TCNT = TCNT + 1;
                end
                if (clear) begin
                    TCNT <= 0;
                end
            end
        end
    end
endmodule
