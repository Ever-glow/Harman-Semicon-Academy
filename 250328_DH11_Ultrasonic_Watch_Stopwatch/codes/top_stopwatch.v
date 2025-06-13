`timescale 1ns / 1ps

module top_stopwatch(
    input clk,
    input btnc, 
    input btnr,
    input btnl,
    input btnu,
    input btnd,
    input [1:0] sw_display,
    input sw_convert,
    input [4:0] control,
    output [3:0] fnd_comm,
    output [7:0] fnd_font,

    output [3:0] led,

    output [5:0] data_sec, 
    output [5:0] data_min, 
    output [4:0] data_hour
);

    wire w_run, w_clear;
    wire run, clear;

    wire [6:0] w_msec, stopwatch_msec, watch_msec;
    wire [5:0] w_sec, stopwatch_sec, watch_sec;
    wire [5:0] w_min, stopwatch_min, watch_min;
    wire [4:0] w_hour, stopwatch_hour, watch_hour; 

    wire w_btn_sec, w_btn_min, w_btn_hour;
    wire w_cu_btn_sec, w_cu_btn_min, w_cu_btn_hour;

    wire sw_mode;

    wire i_btnc_watch, i_btnd_watch, i_btnu_watch, i_btnl_watch;
    wire i_btnc_stw, i_btnr_stw, i_btnl_stw;

    assign led[0] = (sw_display[0]) ? 1'b0 : 1'b1; // STW 초,밀리초
    assign led[1] = (sw_display[0]) ? 1'b1 : 1'b0; // STW 시,분
    assign led[2] = (sw_display[1]) ? 1'b0 : 1'b1; // W 초,밀리초
    assign led[3] = (sw_display[1]) ? 1'b1 : 1'b0; // W 시,분분
     
    assign i_btnc_watch = (~sw_convert & btnc);     // reset 
    assign i_btnd_watch = (~sw_convert & btnd);     // min++
    assign i_btnu_watch = (~sw_convert & btnu);     // sec++
    assign i_btnl_watch = (~sw_convert & btnl);     // hour++

    assign i_btnc_stw = (sw_convert & btnc);    // reset
    assign i_btnr_stw = (sw_convert & btnr);    // clear
    assign i_btnl_stw = (sw_convert & btnl);    // run/stop

    //sw[15]==1 -> stopwatch
    assign {w_msec, w_sec, w_min, w_hour} = (sw_convert) ? 
           {stopwatch_msec, stopwatch_sec, stopwatch_min, stopwatch_hour} :
           {watch_msec, watch_sec, watch_min, watch_hour};

    assign sw_mode = (sw_convert) ? sw_display[0] : sw_display[1];

    assign {data_sec, data_min, data_hour}
         = {w_sec, w_min, w_hour};



    btn_debounce u_btn_db_run(
        .clk(clk),
        .reset(i_btnc_stw),
        .i_btn(i_btnl_stw),
        .o_btn(w_run)
    );

    btn_debounce u_btn_db_clear(
        .clk(clk),
        .reset(i_btnc_stw),
        .i_btn(i_btnr_stw),
        .o_btn(w_clear)
    );

    btn_debounce u_btn_sec(
        .clk(clk),
        .reset(i_btnc_watch),
        .i_btn(i_btnu_watch),
        .o_btn(w_btn_sec)
    );

    btn_debounce u_btn_min(
        .clk(clk),
        .reset(i_btnc_watch),
        .i_btn(i_btnd_watch),
        .o_btn(w_btn_min)
    );

    btn_debounce u_btn_hour(
        .clk(clk),
        .reset(i_btnc_watch),
        .i_btn(i_btnl_watch),
        .o_btn(w_btn_hour)
    );

    stopwatch_cu U_StopWatch_CU(
        .clk(clk),
        .reset(i_btnc_stw),
        .i_btn_run(w_run || (control[3] && sw_convert)),
        .i_btn_clear(w_clear || (control[4] && sw_convert)),
        .o_run(run),
        .o_clear(clear)
    );

    stopwatch_dp u_stopwatch_dp(
        .clk(clk),
        .reset(i_btnc_stw),
        .run(run),
        .clear(clear),
        .msec(stopwatch_msec),
        .sec(stopwatch_sec),
        .min(stopwatch_min),
        .hour(stopwatch_hour)
    );

    watch_cu u_watch_cu(
        .clk(clk),
        .reset(i_btnc_watch),
        .i_btn_sec(w_btn_sec || (control[2] && ~sw_convert)),
        .i_btn_min(w_btn_min || (control[1] && ~sw_convert)),
        .i_btn_hour(w_btn_hour || (control[0] && ~sw_convert)),
        .o_sec(w_cu_btn_sec),
        .o_min(w_cu_btn_min),
        .o_hour(w_cu_btn_hour)
    );

    watch1_dp u_watch1_dp(
        .clk(clk),
        .reset(i_btnc_watch),
        .btn_sec(w_cu_btn_sec),
        .btn_min(w_cu_btn_min),
        .btn_hour(w_cu_btn_hour),
        .o_msec(watch_msec),
        .o_sec(watch_sec),
        .o_min(watch_min),
        .o_hour(watch_hour)
    );

    fnd_controller_time u_fnd_ctrl(
        .clk(clk),
        .reset(btnc),
        .sw_mode(sw_mode),
        .msec(w_msec),
        .sec(w_sec),
        .min(w_min),
        .hour(w_hour),
        .seg(fnd_font),
        .seg_comm(fnd_comm)
    );

endmodule
