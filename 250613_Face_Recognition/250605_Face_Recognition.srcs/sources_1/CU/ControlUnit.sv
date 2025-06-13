`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/06/05 09:55:29
// Design Name: 
// Module Name: ControlUnit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ControlUnit (
    input logic clk,
    input logic reset,
    input logic uart_rx_done,
    input logic [7:0] uart_in,
    input logic [8:0] distance_in,
    input logic measure_done,
    input logic btn_pulse,
    input logic btn_done,
    input logic btn_pass,
    input logic btn_fail,

    output logic capture_trigger,
    output logic hr04_trigger,
    output logic [3:0] fnd_data,
    output logic [7:0] led_out,
    output logic w_en,

    output logic pass_signal,
    output logic fail_signal

);


    logic tick_500ms;
    logic [2:0] approach_count, approach_count_next;
    logic [3:0] PF_count, PF_count_next;
    logic [1:0] is_pass, is_pass_next;
    logic [4:0] fnd_reg, fnd_next;
    logic [7:0] led_reg, led_next;
    logic WEN_REG, WEN_NEXT;

    logic pass_signal_reg, pass_signal_next;
    logic fail_signal_reg, fail_signal_next;

    assign fnd_data = fnd_reg / 2;
    assign led_out = led_reg;
    assign w_en = WEN_REG;

    assign pass_signal = pass_signal_reg;
    assign fail_signal = fail_signal_reg;

    logic [2:0] time_over, time_over_next;
    logic [2:0] fnd_buffer, fnd_buffer_next;

    tick_gen #(
        .FCOUNT(10000_0000 / 2)
    ) u_tick_500ms (
        .clk  (clk),
        .reset(reset),

        .tick(tick_500ms)
    );

    typedef enum {
        IDLE,
        SEND,
        MEASURE,
        IS_APPROACH,
        APPROACH,
        PASSFAIL
    } state_e;

    state_e state, state_next;

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            state <= IDLE;
            approach_count <= 0;
            is_pass <= 0;
            PF_count <= 0;
            fnd_reg <= 0;
            led_reg <= 0;
            WEN_REG <= 0;
            pass_signal_reg <= 0;
            fail_signal_reg <= 0;
            time_over <= 0;
            fnd_buffer <= 0;
        end else begin
            state <= state_next;
            approach_count <= approach_count_next;
            is_pass <= is_pass_next;
            PF_count <= PF_count_next;
            fnd_reg <= fnd_next;
            led_reg <= led_next;
            WEN_REG <= WEN_NEXT;
            pass_signal_reg <= pass_signal_next;
            fail_signal_reg <= fail_signal_next;
            time_over <= time_over_next;
            fnd_buffer <= fnd_buffer_next;
        end
    end

    always_comb begin
        state_next = state;
        hr04_trigger = 0;
        approach_count_next = approach_count;
        PF_count_next = PF_count;
        is_pass_next = is_pass;
        capture_trigger = 0;
        fnd_next = fnd_reg;
        led_next = led_reg;
        WEN_NEXT = WEN_REG;
        pass_signal_next = pass_signal_reg;
        fail_signal_next = fail_signal_reg;
        time_over_next = time_over;
        fnd_buffer_next = fnd_buffer;
        case (state)
            IDLE: begin
                state_next = SEND;
                //hr04_trigger = 1'b1;
                approach_count_next = 0;
                PF_count_next = 0;
                WEN_NEXT = 1'b1;
                pass_signal_next = 0;
                fail_signal_next = 0;
            end
            SEND: begin
                if (tick_500ms) begin
                    state_next = MEASURE;
                    hr04_trigger = 1'b1;
                    approach_count_next = 0;
                    WEN_NEXT = 1'b1;
                    pass_signal_next = 0;
                    fail_signal_next = 0;
                end
            end
            MEASURE: begin
                if (btn_pulse) begin
                    state_next = APPROACH;
                    WEN_NEXT = 0;
                    capture_trigger = 1'b1;
                    approach_count_next = 0;
                end
                if (measure_done) begin
                    if (distance_in < 100) begin
                        state_next = IS_APPROACH;
                        led_next   = 1;
                        fnd_next   = 0;
                    end else state_next = MEASURE;
                end

                if (tick_500ms) hr04_trigger = 1'b1;

            end
            IS_APPROACH: begin
                if (approach_count > 6) begin
                    state_next = APPROACH;
                    WEN_NEXT = 0;
                    capture_trigger = 1'b1;
                    approach_count_next = 0;
                end else begin
                end
                if (tick_500ms) begin
                    hr04_trigger = 1'b1;
                    time_over_next = time_over + 1;
                    if (distance_in < 100) begin
                        led_next = led_reg << 1;
                        approach_count_next = approach_count + 1;
                        fnd_next = fnd_reg + 1;
                    end
                    if (time_over == 5) begin
                        time_over_next = 0;
                        if (fnd_buffer == fnd_reg) begin
                            state_next = SEND;
                            fnd_next   = 0;
                            led_next   = 0;
                        end
                    end

                    if(time_over == 0) fnd_buffer_next = fnd_reg; 
                end
            end
            APPROACH: begin
                led_next = 8'hff;
                fnd_next = 10;
                if (uart_rx_done) begin
                    if ((uart_in == 4)) begin  //fail 
                        is_pass_next = 0;
                        state_next = PASSFAIL;
                        PF_count_next = 0;
                    end else if ((uart_in == 2)) begin  //pass
                        is_pass_next = 1;
                        state_next = PASSFAIL;
                        PF_count_next = 0;
                    end else if (uart_in == 3) begin  //save
                        is_pass_next = 2;
                        state_next = PASSFAIL;
                        PF_count_next = 0;
                    end
                end
                if (btn_done) begin
                    led_next = 8'hf0;
                    state_next = PASSFAIL;
                    PF_count_next = 0;
                    is_pass_next = 3;
                end

            end

            PASSFAIL: begin
                if (PF_count > 8) begin
                    WEN_NEXT = 1'b1;
                    state_next = SEND;
                    PF_count_next = 0;
                    fnd_next = 0;
                    led_next = 0;
                end else begin
                    if (tick_500ms) PF_count_next = PF_count + 1;

                    if ((is_pass == 0)) begin
                        fnd_next = 12;  //fail
                        led_next = 8'hff;
                        fail_signal_next = 1;
                    end else if ((is_pass == 1)) begin
                        fnd_next = 14;  // pass
                        led_next = 8'hff;
                        pass_signal_next = 1;
                    end else if (is_pass == 2) begin
                        fnd_next = 16;  //save
                        led_next = 8'hff;
                    end else if (is_pass == 3) begin
                        if (btn_pass) begin
                            fnd_next = 14;  // pass
                            led_next = 8'hff;
                            pass_signal_next = 1;
                        end
                        if (btn_fail) begin
                            fnd_next = 12;  //fail
                            led_next = 8'hff;
                            fail_signal_next = 1;
                        end
                    end

                end
            end


        endcase
    end

endmodule



