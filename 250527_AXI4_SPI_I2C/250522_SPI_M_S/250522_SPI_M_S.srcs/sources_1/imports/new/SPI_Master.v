`timescale 1ns / 1ps

module SPI_Master (
    // global signals
    input            clk,
    input            reset,
    // external signals
    input            cpol,
    input            cpha,
    input            start,
    input      [7:0] tx_data,
    output     [7:0] rx_data,
    output reg       done,
    output reg       ready,
    // external port
    output           SCLK,
    output           MOSI,
    input            MISO,
    output           SS
);

    localparam IDLE = 0, CP_DELAY = 1, CP0 = 2, CP1 = 3;

    wire r_sclk;

    reg [1:0] state, state_next;
    reg [7:0] temp_tx_data_reg, temp_tx_data_next;
    reg [5:0] sclk_counter_next, sclk_counter_reg;
    reg [2:0] bit_counter_next, bit_counter_reg;
    reg [7:0] temp_rx_data_reg, temp_rx_data_next;
    reg       ss_reg, ss_next;
    reg [2:0] byte_cnt_reg, byte_cnt_next;

    assign MOSI = temp_tx_data_reg[7];
    assign rx_data = temp_rx_data_reg;
    assign SS = ss_reg;

    assign bit_counter = bit_counter_reg;

    assign r_sclk = ((state == CP1) && ~cpha) || 
                    ((state == CP0) && cpha);
    assign SCLK = cpol ? ~r_sclk : r_sclk;
    // sclk는 cpha, cpol 및 상태에 따라 결정

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state            <= IDLE;
            temp_tx_data_reg <= 0;
            temp_rx_data_reg <= 0;
            sclk_counter_reg <= 0;
            bit_counter_reg  <= 0;
            ss_reg           <= 1;
            byte_cnt_reg     <= 0;
        end else begin
            state            <= state_next;
            temp_tx_data_reg <= temp_tx_data_next;
            temp_rx_data_reg <= temp_rx_data_next;
            sclk_counter_reg <= sclk_counter_next;
            bit_counter_reg  <= bit_counter_next;
            ss_reg           <= ss_next;
            byte_cnt_reg     <= byte_cnt_next;
        end
    end

    always @(*) begin
        state_next        = state;
        ready             = 0;
        done              = 0;
        temp_tx_data_next = temp_tx_data_reg;
        temp_rx_data_next = temp_rx_data_reg;
        sclk_counter_next = sclk_counter_reg;
        bit_counter_next  = bit_counter_reg;
        ss_next           = ss_reg;
        byte_cnt_next     = byte_cnt_reg;
        case (state)
            IDLE: begin
                temp_tx_data_next = 0;
                done              = 0;
                ready             = 1;
                ss_next           = 1;
                byte_cnt_next     = 0;
                if (start) begin
                    state_next        = cpha ? CP_DELAY : CP0;
                    temp_tx_data_next = tx_data;
                    ready             = 0;
                    sclk_counter_next = 0;
                    bit_counter_next  = 0;
                    ss_next           = 0;
                    byte_cnt_next     = 0;
                end
            end
            CP_DELAY: begin
                ss_next           = 0;
                if (sclk_counter_reg == 49) begin
                    sclk_counter_next = 0;
                    state_next = CP0;
                end else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end
            CP0: begin // falling
                ss_next           = 0;
                if (bit_counter_reg == 0 && sclk_counter_reg == 0) temp_tx_data_next = tx_data;
                if (sclk_counter_reg == 49) begin
                    temp_rx_data_next = {temp_rx_data_reg[6:0], MISO};
                    sclk_counter_next = 0;
                    state_next = CP1;
                end else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end
            CP1: begin // rising
                ss_next           = 0;
                if (bit_counter_reg == 7) done = 1;
                if (sclk_counter_reg == 49) begin
                    sclk_counter_next = 0;
                    temp_tx_data_next = {temp_tx_data_reg[6:0], 1'b0};
                    if (bit_counter_reg == 7) begin
                        bit_counter_next = 0;
                        if (byte_cnt_reg == 4) begin
                            ss_next = 1; // 4byte 보내면 SS 해제
                            byte_cnt_next = 0;
                            state_next = IDLE;
                        end
                        else begin
                            byte_cnt_next = byte_cnt_reg + 1;
                            state_next    = cpha ? CP_DELAY : CP0;
                        end
                    end
                    else begin
                        bit_counter_next = bit_counter_reg + 1;
                        state_next = CP0;
                    end
                end
                else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end
        endcase
    end

endmodule
