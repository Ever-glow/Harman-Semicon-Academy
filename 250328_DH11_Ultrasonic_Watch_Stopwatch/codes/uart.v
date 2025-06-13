`timescale 1ns / 1ps

module TOP_UART (
    input clk,
    input reset,
    input rx,
    input tx_start,
    input [7:0] tx_data,
    output tx_done,
    output [7:0] rx_data,
    output rx_done,
    output tx
   // output [7:0] seg,
   // output [3:0] seg_comm
);
    wire w_rx_done;
    wire [7:0] w_rx_data;

    uart U_UART(
        .clk(clk),
        .reset(reset),
        .btn_start(tx_start),
        .tx_data_in(tx_data),
        .tx_done(tx_done),
        .tx(tx),
        .rx(rx),
        .rx_done(rx_done),
        .rx_data(rx_data)
    );
/*
    fnd_controller U_FND_CON(
        .data(w_rx_data),
        .clk(clk),
        .reset(reset),
        .seg(seg),
        .seg_comm(seg_comm)
    );  
*/
endmodule

module uart(
    input clk,
    input reset,
    // tx
    input btn_start,
    input [7:0] tx_data_in,
    output tx_done,
    output tx,
    // rx
    input rx,
    output rx_done,
    output [7:0] rx_data
);

    wire w_tick;

    baud_tick_gen U_BAUD_Tick_Gen(
        .clk(clk),
        .reset(reset),
        .baud_tick(w_tick)
    );

    uart_tx U_UART_TX(
        .clk(clk),
        .reset(reset),
        .tick(w_tick),
        .start_trigger(btn_start),
        .data_in(tx_data_in),
        .o_tx_done(tx_done),
        .o_tx(tx)
    );

    uart_rx U_UART_RX(
            .clk(clk),
            .reset(reset),
            .tick(w_tick),
            .rx(rx),
            .rx_done(rx_done),
            .rx_data(rx_data)
        );


endmodule


module baud_tick_gen(
    input clk,
    input reset,
    output baud_tick
);

    parameter BAUD_RATE = 9600; // BAUD_RATE = 19200;
    localparam BAUD_COUNT = 100_000_000 /  BAUD_RATE / 16;  // tick이 16배 빨라짐

    reg [$clog2(BAUD_COUNT)-1:0] count_reg, count_next;
    reg tick_reg, tick_next;
    assign baud_tick = tick_reg;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            count_reg <= 0;
            tick_reg <= 0;
        end else begin
            count_reg <= count_next;
            tick_reg <= tick_next;
        end
    end

    always @(*) begin
        count_next = count_reg;
        tick_next = tick_reg;
        if (count_reg == BAUD_COUNT - 1) begin
            count_next = 0;
            tick_next = 1'b1;
        end else begin
            count_next = count_reg + 1;
            tick_next = 1'b0;
        end
    end
    
endmodule

module uart_rx (
    input clk,
    input reset, 
    input tick,
    input rx,
    output rx_done,
    output [7:0] rx_data
);
    localparam IDLE = 0, START = 1, DATA = 2, STOP = 3;
    reg [1:0] state, next;
    reg rx_done_reg, rx_done_next;
    reg [2:0] bit_count_reg, bit_count_next;
    reg [4:0] tick_count_reg, tick_count_next;
    reg [7:0] rx_data_reg, rx_data_next;

    assign rx_done = rx_done_reg;
    assign rx_data = rx_data_reg;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state <= 0;
            rx_done_reg <= 0; 
            rx_data_reg <= 0;
            bit_count_reg <= 0;
            tick_count_reg <= 0;  
        end
        else begin
            state <= next;
            rx_done_reg <= rx_done_next;
            rx_data_reg <= rx_data_next;
            bit_count_reg <= bit_count_next;
            tick_count_reg <= tick_count_next;
        end
    end

    always @(*) begin
        next = state;
        tick_count_next = tick_count_reg;
        bit_count_next = bit_count_reg;
        rx_data_next = rx_data_reg;
        rx_done_next = rx_done_reg;
        rx_done_next = 1'b0;
        case (state)
            IDLE: begin
                tick_count_next = 0;
                bit_count_next = 0;
                rx_done_next = 1'b0;
                if (rx == 1'b0) begin
                    next = START;
                end
            end
            START: begin
                if (tick == 1'b1) begin
                    if (tick_count_reg == 7) begin
                        next = DATA;
                        tick_count_next = 0;
                    end
                    else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
            DATA: begin
                if (tick == 1'b1) begin
                    if (tick_count_reg == 15) begin
                        rx_data_next[bit_count_reg] = rx;       // read data
                        if (bit_count_reg == 7) begin
                            next = STOP;
                            tick_count_next = 0;
                            bit_count_next = 0;
                        end
                        else begin
                            next = DATA;
                            bit_count_next = bit_count_reg + 1;
                            tick_count_next = 0;
                        end
                    end
                    else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
            STOP: begin
                if (tick == 1'b1) begin
                    if (tick_count_reg == 23) begin
                        next = IDLE;
                        rx_done_next = 1'b1;
                    end
                    else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
        endcase
    end
    
endmodule


module uart_tx(
    input clk,
    input reset,
    input tick,
    input start_trigger,
    input [7:0] data_in,
    output o_tx_done,
    output o_tx
);


    parameter IDLE = 0, SEND = 1, START = 2, DATA = 3, STOP = 4;
    
    reg [2:0] state, next;
    reg tx_reg, tx_next;
    reg tx_done_reg, tx_done_next;
    reg [2:0] bit_count_reg, bit_count_next;
    reg [3:0] tick_count_reg, tick_count_next;
    
    assign o_tx_done = tx_done_reg;
    assign o_tx = tx_reg;
    
    reg [7:0] temp_data_reg, temp_data_next;

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state <= 0;
            tx_reg <= 1'b1; // Uart tx line을 초기에 항상 1로 만들기 위함
            tx_done_reg <= 0;
            bit_count_reg <= 0;
            tick_count_reg <= 0;
            temp_data_reg <= 0;
        end else begin
            state <= next;
            tx_reg <= tx_next;
            tx_done_reg <= tx_done_next;
            bit_count_reg <= bit_count_next;
            tick_count_reg <= tick_count_next;
            temp_data_reg <= temp_data_next;
        end
    end

    always @(*) begin
        next = state;
        tx_next = tx_reg;
        tx_done_next = tx_done_reg;
        bit_count_next = bit_count_reg;
        tick_count_next = tick_count_reg;
        temp_data_next = temp_data_reg;
        case (state)
            IDLE: begin
                tx_next = 1'b1;
                tx_done_next = 1'b0;
                tick_count_next = 4'h0;
                if (start_trigger) begin
                    next = START;
                    // start trigger 순간 data buffering
                    temp_data_next = data_in;
                end
            end
            SEND: begin
                if (tick) begin
                    next = START;
                end
            end
            START: begin
                tx_next = 1'b0; // 출력을 0으로 유지. 여기서 떨궈도 되고 다음 상태 가서 떨궈도 됨. (한 clk정도 차이)
                tx_done_next = 1'b1; // SEND에서 오자마자 올려줌
                if (tick) begin
                    if (tick_count_reg == 15) begin
                        next = DATA;
                        tick_count_next = 1'b0;
                        bit_count_next = 1'b0;
                    end else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
            DATA: begin
               // tx_next = data_in[bit_count_reg]; 
                tx_next = temp_data_reg[bit_count_reg];  
                if (tick) begin
                    if (tick_count_reg == 15) begin
                        tick_count_next = 0; 
                        if (bit_count_reg == 7) begin
                            next = STOP;
                            bit_count_next = 0; 
                        end else begin
                            next = DATA;
                            bit_count_next = bit_count_reg + 1;
                        end
                    end else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end
            STOP: begin
                tx_next = 1'b1;
                if (tick) begin
                    if (tick_count_reg == 15) begin
                        next = IDLE;
                    end else begin
                        tick_count_next = tick_count_reg + 1;
                    end
                end
            end 
            // default: 
        endcase
    end

endmodule