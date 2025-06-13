`timescale 1ns / 1ps
module top_uart2stopwatch(
    input clk, reset,
    input [7:0] data,
    input data_valid,
    output [4:0] control
);

uart2stopwatch U_uart2stopwatch(
    .clk(clk),
    .reset(reset),
    .data(data),
    .data_valid(data_valid),
    .control(control)
);

endmodule

module uart2stopwatch(
    input clk,
    input reset,
    input [7:0] data,
    input data_valid,
    output [4:0] control
    // control[0]: hour_con,
    // control[1]: min_con,
    // control[2]: sec_con,
    // control[3]: run_con,
    // control[4]: clear_con
);

    reg [4:0] control_reg, control_next;
    assign  control = control_reg;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            control_reg <= 0;
        end
        else begin
            control_reg <= control_next;
        end
    end

    always @(*) begin
        control_next = 5'b0;
        if (data_valid) begin
            case(data)
                8'h48, 8'h68: control_next[0] = 1'b1; // H, h
                8'h4D, 8'h6D: control_next[1] = 1'b1; // M, m
                8'h53, 8'h73: control_next[2] = 1'b1; // S, s
                8'h52, 8'h72: control_next[3] = 1'b1; // R, r
                8'h43, 8'h63: control_next[4] = 1'b1; // C, c
                default:      control_next = 5'b0;
            endcase
        end
        else
            control_next = 5'b0;
    end

endmodule

