//Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2020.2 (win64) Build 3064766 Wed Nov 18 09:12:45 MST 2020
//Date        : Sat May 24 20:13:48 2025
//Host        : DESKTOP-7CFQ9ND running 64-bit major release  (build 9200)
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (MISO,
    MOSI,
    SCLK,
    SS,
    reset,
    sw,
    sys_clock,
    usb_uart_rxd,
    usb_uart_txd);
  input MISO;
  output MOSI;
  output SCLK;
  output SS;
  input reset;
  input [7:0]sw;
  input sys_clock;
  input usb_uart_rxd;
  output usb_uart_txd;

  wire MISO;
  wire MOSI;
  wire SCLK;
  wire SS;
  wire reset;
  wire [7:0]sw;
  wire sys_clock;
  wire usb_uart_rxd;
  wire usb_uart_txd;

  design_1 design_1_i
       (.MISO(MISO),
        .MOSI(MOSI),
        .SCLK(SCLK),
        .SS(SS),
        .reset(reset),
        .sw(sw),
        .sys_clock(sys_clock),
        .usb_uart_rxd(usb_uart_rxd),
        .usb_uart_txd(usb_uart_txd));
endmodule
