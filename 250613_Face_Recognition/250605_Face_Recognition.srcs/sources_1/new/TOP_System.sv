`timescale 1ns / 1ps


module TOP_System(
    input logic rx_in,
    input logic echo,
    output logic hr_trigger,
    output logic [7:0] led_out,

    output logic [3:0] fndcomm,
    output logic [7:0] fndfont,
    //----------------------//
    input  logic       clk,
    input  logic       reset,
    input  logic       btn,
    input  logic       btn_cap,

    input  logic       btn_pass,
    input  logic       btn_fail,
    // filter signals
    input  logic       sw_red,
    input  logic       sw_green,
    input  logic       sw_blue,
    input  logic       sw_gray,
    input  logic       sw_upscale,
    // ov7670 signals
    output logic       ov7670_xclk,
    input  logic       ov7670_pclk,
    input  logic       ov7670_href,
    input  logic       ov7670_v_sync,
    input  logic [7:0] ov7670_data,
    output logic       ov7670_scl,
    output logic       ov7670_sda,
    // export
    output logic       h_sync,
    output logic       v_sync,
    output logic [3:0] red_port,
    output logic [3:0] green_port,
    output logic [3:0] blue_port,
    output logic       tx,
    output logic [6:0] led,

    //
    input  logic        sw_gaussian,
    input  logic        sw_filter
    );

    logic capture_trigger, we_cu, btn_pulse, btn_done, btn_done_pass, btn_done_fail;

    logic pass_signal, fail_signal;

    OV7670_VGA_Display U_OV7670_VGA(
        .*,
        .capture_trigger(capture_trigger), // to use 2 board(hc04)
        .we_cu(we_cu),            // to use 2 board(hc04)
        .pass_signal(pass_signal),
        .fail_signal(fail_signal)
    );

    top_ctrl U_CU(
        .*,
        .capture_trigger(capture_trigger),
        .w_en(we_cu),
        .btn_pulse(btn_pulse),
        .btn_done(btn_done),
        .btn_pass(btn_done_pass),
        .btn_fail(btn_done_fail),
        .pass_signal(pass_signal),
        .fail_signal(fail_signal)
    );

    button_debounce_edge U_btn_deb1 (
        .clk      (clk),
        .reset    (reset),
        .btn_in   (btn),
        .btn_pulse(btn_pulse)
    );

    button_debounce_edge U_btn_deb2 (
        .clk      (clk),
        .reset    (reset),
        .btn_in   (btn_cap),
        .btn_pulse(btn_done)
    );

    button_debounce_edge U_btn_deb3 (
        .clk      (clk),
        .reset    (reset),
        .btn_in   (btn_pass),
        .btn_pulse(btn_done_pass)
    );

    button_debounce_edge U_btn_deb4 (
        .clk      (clk),
        .reset    (reset),
        .btn_in   (btn_fail),
        .btn_pulse(btn_done_fail)
    );

endmodule

