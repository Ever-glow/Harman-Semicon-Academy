module UART_Periph (
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
    input  logic        rx,
    output logic        tx
);

    logic [7:0] uart_tx_data;
    logic       uart_tx_done;
    logic [7:0] uart_rx_data;
    logic       uart_rx_done;
    logic       uart_tx_start;

    TOP_UART U_UART (
        .clk     (PCLK),
        .reset   (PRESET),
        .rx      (rx),
        .tx_start(uart_tx_start),
        .tx_data (uart_tx_data),
        .tx_done (uart_tx_done),
        .rx_data (uart_rx_data),
        .rx_done (uart_rx_done),
        .tx      (tx)
    );

    APB_SlaveIntf_UART U_FIFO_for_uart (
        // global signals

        .PCLK  (PCLK),
        .PRESET(PRESET),

        // APB Interface Signals

        .PADDR(PADDR),
        .PWDATA(PWDATA),
        .PWRITE(PWRITE),
        .PENABLE(PENABLE),
        .PSEL(PSEL),
        .PRDATA(PRDATA),
        .PREADY(PREADY),

        // UART RX Interface

        .uart_rx_data(uart_rx_data),
        .uart_rx_done(uart_rx_done),

        // UART TX Interface

        .uart_tx_data (uart_tx_data),
        .uart_tx_start(uart_tx_start),
        .uart_tx_done (uart_tx_done)
    );

endmodule
