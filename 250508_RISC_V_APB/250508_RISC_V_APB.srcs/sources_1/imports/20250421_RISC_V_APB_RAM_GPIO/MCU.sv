`timescale 1ns / 1ps

module MCU (
    input  logic       clk,
    input  logic       reset,
    inout  logic [7:0] GPIOA,
    inout  logic [7:0] GPIOB,
    inout  logic [7:0] GPIOC,
    inout  logic [7:0] GPIOD,
    output logic [3:0] fndComm,
    output logic [7:0] fndFont,
    input  logic       rx,
    output logic       tx,
    input  logic       ECHO,
    output logic       TRIG,
    inout  wire        dht_io
);
    // global signals
    logic        PCLK;
    logic        PRESET;
    // APB Interface Signals
    logic [31:0] PADDR;
    logic [31:0] PWDATA;
    logic        PWRITE;
    logic        PENABLE;
    logic        PSEL_RAM;
    logic        PSEL_GPIOA;
    logic        PSEL_GPIOB;
    logic        PSEL_GPIOC;
    logic        PSEL_GPIOD;
    logic        PSEL_FND;
    logic        PSEL_US;
    logic        PSEL_DHT;
    logic        PSEL_UART;
    logic        PSEL_TIMER;

    logic [31:0] PRDATA_RAM;
    logic [31:0] PRDATA_GPIOA;
    logic [31:0] PRDATA_GPIOB;
    logic [31:0] PRDATA_GPIOC;
    logic [31:0] PRDATA_GPIOD;
    logic [31:0] PRDATA_FND;
    logic [31:0] PRDATA_US;
    logic [31:0] PRDATA_DHT;
    logic [31:0] PRDATA_UART;
    logic [31:0] PRDATA_TIMER;

    logic        PREADY_RAM;
    logic        PREADY_GPIOA;
    logic        PREADY_GPIOB;
    logic        PREADY_GPIOC;
    logic        PREADY_GPIOD;
    logic        PREADY_FND;
    logic        PREADY_US;
    logic        PREADY_DHT;
    logic        PREADY_UART;
    logic        PREADY_TIMER;

    // CPU - APB_Master Signals
    // Internal Interface Signals
    logic        transfer;  // trigger signal
    logic        ready;
    logic [31:0] addr;
    logic [31:0] wdata;
    logic [31:0] rdata;
    logic        write;  // 1:write, 0:read
    logic        dataWe;
    logic [31:0] dataAddr;
    logic [31:0] dataWData;
    logic [31:0] dataRData;

    // ROM Signals
    logic [31:0] instrCode;
    logic [31:0] instrMemAddr;
 
    assign PCLK = clk;
    assign PRESET = reset;
    assign addr = dataAddr;
    assign wdata = dataWData;
    assign dataRData = rdata;
    assign write = dataWe;

    rom U_ROM (
        .addr(instrMemAddr),
        .data(instrCode)
    );

    RV32I_Core U_Core (.*);
 
    APB_Master U_APB_Master (
        .*,
        .PSEL0  (PSEL_RAM),
        .PSEL1  (PSEL_GPIOA),
        .PSEL2  (PSEL_GPIOB),
        .PSEL3  (PSEL_GPIOC),
        .PSEL4  (PSEL_GPIOD),
        .PSEL5  (PSEL_FND),
        .PSEL6  (PSEL_US),
        .PSEL7  (PSEL_DHT),
        .PSEL8  (PSEL_UART),
        .PSEL9  (PSEL_TIMER),
        .PRDATA0(PRDATA_RAM),
        .PRDATA1(PRDATA_GPIOA),
        .PRDATA2(PRDATA_GPIOB),
        .PRDATA3(PRDATA_GPIOC),
        .PRDATA4(PRDATA_GPIOD),
        .PRDATA5(PRDATA_FND),
        .PRDATA6(PRDATA_US),
        .PRDATA7(PRDATA_DHT),
        .PRDATA8(PRDATA_UART),
        .PRDATA9(PRDATA_TIMER),
        .PREADY0(PREADY_RAM),
        .PREADY1(PREADY_GPIOA),
        .PREADY2(PREADY_GPIOB),
        .PREADY3(PREADY_GPIOC),
        .PREADY4(PREADY_GPIOD),
        .PREADY5(PREADY_FND),
        .PREADY6(PREADY_US),
        .PREADY7(PREADY_DHT),
        .PREADY8(PREADY_UART),
        .PREADY9(PREADY_TIMER)
    );

    ram U_RAM (
        .*,
        .PSEL  (PSEL_RAM),
        .PRDATA(PRDATA_RAM),
        .PREADY(PREADY_RAM)
    );

    GPIO_Periph U_GPIOA (
        .*,
        .PSEL   (PSEL_GPIOA),
        .PRDATA (PRDATA_GPIOA),
        .PREADY (PREADY_GPIOA),
        // export signals
        .inoutPort(GPIOA)
    );

    GPIO_Periph U_GPIOB (
        .*,
        .PSEL  (PSEL_GPIOB),
        .PRDATA(PRDATA_GPIOB),
        .PREADY(PREADY_GPIOB),
        // inport signals
        .inoutPort(GPIOB)
    );

    GPIO_Periph U_GPIOC (
        .*,
        .PSEL     (PSEL_GPIOC),
        .PRDATA   (PRDATA_GPIOC),
        .PREADY   (PREADY_GPIOC),
        // inoutport signals
        .inoutPort(GPIOC)
    );

    GPIO_Periph U_GPIOD (
        .*,
        .PSEL     (PSEL_GPIOD),
        .PRDATA   (PRDATA_GPIOD),
        .PREADY   (PREADY_GPIOD),
        // inoutport signals
        .inoutPort(GPIOD)
    );

    FND_Periph U_FND (
        .*,
        .PSEL  (PSEL_FND),
        .PRDATA(PRDATA_FND),
        .PREADY(PREADY_FND),
        // inport signals
        .fndComm(fndComm),
        .fndFont(fndFont)
    );

    Ultrasonic_Periph U_US (
        .*,
        .PSEL(PSEL_US),
        .PRDATA(PRDATA_US),
        .PREADY(PREADY_US),
        .TRIG(TRIG),
        .ECHO(ECHO)
    );

    DHT11_Periph U_DHT (
        .*,
        .PSEL(PSEL_DHT),
        .PRDATA(PRDATA_DHT),
        .PREADY(PREADY_DHT),
        .dht_io(dht_io)
    );

    UART_Periph U_UART(
        .*,
        .PSEL(PSEL_UART),
        .PRDATA(PRDATA_UART),
        .PREADY(PREADY_UART),
        .rx(rx),
        .tx(tx)
    );

    Timer_Periph U_TIMER (
        .*,
        .PSEL(PSEL_TIMER),
        .PRDATA(PRDATA_TIMER),
        .PREADY(PREADY_TIMER)
    );

endmodule
