`timescale 1ns / 1ps

module top_distance (
    input clk, reset,
    input start,
    input echo,
    output trig,
    output [8:0] distance,
    output done
);

    wire w_tick;

    control_unit U_CU(
        .clk(clk),
        .reset(reset),
        .start(start),
        .echo(echo),
        .tick(w_tick),
        .trig(trig),
        .distance(distance),
        .done(done)
    );

    tick_1us U_TICK_1US(
        .clk(clk),
        .reset(reset),
        .o_clk(w_tick)
    );
endmodule



module control_unit(
    input clk, reset,
    input start,
    input echo,
    input tick,
    output trig,
    output [8:0] distance,
    output done
    );

    parameter PULSE_10US = 1000;

    reg [3:0] state, next_state;
    reg trig_reg, trig_next, done_reg, done_next;
    reg [8:0] distance_reg, distance_next;
    reg [31:0] counter_reg, counter_next, echo_counter_reg, echo_counter_next;

    assign trig = trig_reg;
    assign distance = distance_reg;
    assign done = done_reg;


    parameter IDLE = 1, TRIGGER = 2, WAIT_ECHO_H = 3,
              MEASURE_ECHO = 4, WAIT_BETWEEN = 5, DONE = 6;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state            <= IDLE;
            trig_reg         <= 1'b0;
            distance_reg     <= 8'd0;
            done_reg         <= 1'b0;
            counter_reg      <= 32'd0;
            echo_counter_reg <= 32'd0;
        end 
        else begin
            state            <= next_state;
            trig_reg         <= trig_next;
            distance_reg     <= distance_next;
            done_reg         <= done_next;
            counter_reg      <= counter_next;
            echo_counter_reg <= echo_counter_next;
        end
    end

    always @(*) begin
        next_state = state;
        trig_next = trig_reg;
        done_next = done_reg;
        echo_counter_next = echo_counter_reg;
        counter_next = counter_reg;
        distance_next = distance_reg;
        case (state)
            IDLE            : begin
                trig_next = 0;
                done_next = 0;
                echo_counter_next = 0;
                counter_next = 0;
                distance_next = 0;
                if (start) begin
                    next_state = TRIGGER;
                end
                else begin
                    next_state = state;
                end
            end
            TRIGGER       : begin
                trig_next = 1'b1;
                if (counter_reg < PULSE_10US) begin
                    counter_next = counter_reg + 1;
                end 
                else begin
                    counter_next = 0;
                end

                if (counter_reg >= PULSE_10US) begin
                    next_state = WAIT_ECHO_H;
                    trig_next = 0;
                    counter_next = 0;
                end
                else begin
                    next_state = state;
                end
            end
            WAIT_ECHO_H     : begin
                if (echo) begin
                    next_state = MEASURE_ECHO;
                end
                else begin
                    next_state = state;
                end
            end
            MEASURE_ECHO    : begin
                if (echo) begin
                    if (tick) begin
                        echo_counter_next = echo_counter_reg + 1;
                    end
                    else begin
                        echo_counter_next = echo_counter_reg;
                    end
                end
                if (!echo) begin
                    next_state = WAIT_BETWEEN;
                end
            end
            WAIT_BETWEEN    : begin
                if (counter_reg < 6_000_000) begin //60ms
                    counter_next = counter_reg + 1;
                end
                if (counter_reg >= 6_000_000) begin
                    next_state = DONE;
                end
            end
            DONE            : begin
                distance_next = echo_counter_reg / 58;
                done_next = 1'b1;
                next_state = IDLE;
            end
        endcase
    end

endmodule

module tick_1us (
    input clk, reset,
    output o_clk
);

    parameter COUNT = 100;
    reg [$clog2(COUNT)-1:0] r_counter;
    reg r_clk;
    assign o_clk = r_clk;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_counter <= 0;
            r_clk <= 1'b0;
        end
        else begin
            if (r_counter == COUNT - 1) begin
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