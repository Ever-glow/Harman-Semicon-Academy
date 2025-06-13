`timescale 1ns / 1ps

module TOP_UART_FIFO(
    input clk, reset, rx,
    input [8:0] dist,
    input [2:0] sw,
    input [1:0] mode,
    input [5:0] sec,
    input [5:0] min,
    input [4:0] hour,
    input [7:0] humid,
    input [7:0] temp,
    output tx,
    output [7:0] out_data,
    output done
    );


    wire [7:0] w_fifo_rx_rdata;
    wire w_fifo_rx_empty;

    wire [7:0] w_rx_wdata; // uart rxdata to fifo rx wdata
    wire w_rx_wr;          // uart rxdone to fifo rx wr

    wire w_fifo_tx_full;
    wire w_fifo_tx_empty;

    wire [7:0] w_tx_rdata; // uart txdata from fifo tx rdata
    wire w_tx_done;        // uart txdone to   fifo tx rd

    assign out_data = w_fifo_rx_rdata;
    assign done = ~w_fifo_rx_empty;

    wire pg_done, pg_busy;
    wire [7:0] pg_tx_data;
    wire pg_tx_wr;

    wire [7:0] w_fifo_tx_data; // FIFO TX로 들어가는 데이터, mode에 따라 결정
    wire w_fifo_tx_wr;
    wire w_fifo_tx_rd;

    reg [7:0] r_w_fifo_tx_data;
    reg r_w_fifo_tx_wr;
    reg r_w_fifo_tx_rd;

    assign w_fifo_tx_data = r_w_fifo_tx_data;
    assign w_fifo_tx_wr = r_w_fifo_tx_wr;
    assign w_fifo_tx_rd = r_w_fifo_tx_rd;

    always @(*) begin
        r_w_fifo_tx_data = w_fifo_rx_rdata; 
        r_w_fifo_tx_wr   = ~w_fifo_rx_empty; 
        r_w_fifo_tx_rd   = ~w_tx_done & ~w_fifo_tx_empty;
        case (mode)
            2'd0, 2'd1: begin // stopwatch or watch
                r_w_fifo_tx_data = w_fifo_rx_rdata; 
                r_w_fifo_tx_wr   = ~w_fifo_rx_empty; 
                r_w_fifo_tx_rd   = ~w_tx_done & ~w_fifo_tx_empty; 
            end
            2'd2: begin // ultrasonic
                r_w_fifo_tx_data = pg_tx_data; 
                r_w_fifo_tx_wr   = pg_busy; 
                r_w_fifo_tx_rd   = ~w_tx_done&~pg_busy;
            end
            2'd3: begin // humid&temp
                r_w_fifo_tx_data = w_fifo_rx_rdata; 
                r_w_fifo_tx_wr   = ~w_fifo_rx_empty; 
                r_w_fifo_tx_rd   = ~w_tx_done & ~w_fifo_tx_empty;
            end
        endcase
    end

    TOP_UART U_UART(
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .tx_start(~w_fifo_tx_empty),
        .tx_data(w_tx_rdata),
        .tx_done(w_tx_done),
        .rx_data(w_rx_wdata),
        .rx_done(w_rx_wr),
        .tx(tx)
    );

    fifo U_FIFO_TX(
        .clk(clk),
        .reset(reset),
        .wdata(w_fifo_tx_data),
        .wr(w_fifo_tx_wr),
        .full(w_fifo_tx_full),
        .rdata(w_tx_rdata),
        .rd(w_fifo_tx_rd),
        .empty(w_fifo_tx_empty)
    );
    

    fifo U_FIFO_RX(
        .clk(clk),
        .reset(reset),
        .wdata(w_rx_wdata),
        .wr(w_rx_wr),
        .full(),
        .rdata(w_fifo_rx_rdata),
        .rd(~w_fifo_tx_full&~w_fifo_rx_empty),
        .empty(w_fifo_rx_empty)
    );

    print_gen U_DIST_PRINT_GEN(
        .clk(clk),
        .reset(reset),
        .start(~w_fifo_rx_empty),
        .distance_bin(dist),
        .tx_data(pg_tx_data),
        .tx_wr(pg_tx_wr),
        .done(pg_done),
        .busy(pg_busy)
    );

endmodule