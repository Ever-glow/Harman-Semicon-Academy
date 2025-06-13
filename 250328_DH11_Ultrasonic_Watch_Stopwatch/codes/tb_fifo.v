`timescale 1ns / 1ps

module tb_fifo();

    reg clk, reset, rx;
    wire tx;
    wire [7:0] out_data;

    wire [7:0] w_fifo_rx_rdata;
    wire w_fifo_rx_empty;

    wire [7:0] w_rx_wdata; // uart rxdata to fifo rx wdata
    wire w_rx_wr;          // uart rxdone to fifo rx wr

    wire w_fifo_tx_full;
    wire w_fifo_tx_empty;

    wire [7:0] w_tx_rdata; // uart txdata from fifo tx rdata
    wire w_tx_done;

    reg btnd, btnl, btnr, btnu;
    reg [1:0] sw_display;
    reg sw_convert;
    wire [3:0] fnd_comm;
    wire [7:0] fnd_font;
    wire [3:0] led;

    TOP_UART_STOPWATCH DUT(
    // stopwatch I/O
    .clk(clk),
    .reset(reset),
    .btnr(btnr),
    .btnl(btnl),
    .btnu(btnu),
    .btnd(btnu),
    .sw_display(sw_display),
    .sw_convert(sw_convert),
    .fnd_comm(fnd_comm),
    .fnd_font(fnd_font),
    .led(led),
    
    // uart fifo I/O
    .rx(rx),
    .tx(tx)
    );

    TOP_UART_FIFO U_UART_FIFO(
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .tx(tx),
        .out_data(out_data)
    );

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
        .wdata(w_fifo_rx_rdata),
        .wr(~w_fifo_rx_empty),
        .full(w_fifo_tx_full),
        .rdata(w_tx_rdata),
        .rd(~w_tx_done&~w_fifo_tx_empty),
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

    always #5 clk = ~clk;

    reg [7:0] test_data [0:15]; // 테스트 데이터 16개
    integer i;

    initial begin
        clk = 0;
        reset = 1;
        rx = 1; // 기본적으로 UART RX는 idle (1)
        #20;
        reset = 0;
        
        test_data[0]  = 8'h52;
        test_data[1]  = 8'h72;
        test_data[2]  = 8'h43;
        test_data[3]  = 8'h63;
        test_data[4]  = 8'h48;
        test_data[5]  = 8'h68;
        test_data[6]  = 8'h4D;
        test_data[7]  = 8'h6D;
        test_data[8]  = 8'h53;
        test_data[9]  = 8'h73;
        test_data[10] = 8'hFF;
        test_data[11] = 8'hFF;
        test_data[12] = 8'hFF;
        test_data[13] = 8'hFF;
        test_data[14] = 8'hFF;
        test_data[15] = 8'hFF;

        #50;

        // 데이터 송신 (UART RX -> FIFO RX)
        for (i = 0; i < 16; i = i + 1) begin
            send_data(test_data[i]);
            #20000; // 데이터 처리 시간 대기 (UART 비트 시간 고려)
        end
        #600000; // FIFO에서 TX로 데이터가 흘러가는 시간 대기

    end



    task send_data(input [7:0] data);
        integer i;
        begin
            $display("Sending data: %h", data);
            rx = 0; // Start bit
            #(10 * 10417);  // 9600bps 기준
            for (i = 0; i < 8; i = i + 1) begin
                rx = data[i]; 
                #(10 * 10417);
            end
            rx = 1; // Stop bit
            #(10 * 10417);

            $display("Data sent: %h", data);
        end
    endtask
endmodule
