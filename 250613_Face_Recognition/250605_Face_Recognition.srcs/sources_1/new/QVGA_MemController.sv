`timescale 1ns / 1ps



module QVGA_MemController (
    input  logic        clk,
    input  logic [ 9:0] x_pixel,
    input  logic [ 9:0] y_pixel,
    input  logic        DE,
    output logic        rclk,
    output logic        d_en,
    output logic [14:0] rAddr,
    input  logic [15:0] rData,
    input  logic [15:0] rData_cap,
    output logic [ 3:0] red_port,
    output logic [ 3:0] green_port,
    output logic [ 3:0] blue_port,

    input logic pass_signal,
    input logic fail_signal
);

    logic display_live;
    logic display_capture;
    logic display_blank;
    logic [15:0] selected_pixel;

    assign rclk = clk;

    assign display_live = (x_pixel < 160 && y_pixel < 120);
    assign display_capture = (x_pixel >= 160) && (x_pixel < 320) && (y_pixel < 120);
    assign display_blank = (y_pixel >= 120);

    //assign d_en = display_live || display_capture || display_blank;

    assign rAddr = display_live    ? (y_pixel * 160 + x_pixel) 
                 : display_capture ? (y_pixel * 160 + (x_pixel - 160))
                 : 0;

    assign selected_pixel = display_live      ? rData 
                          : display_capture ? rData_cap
                          : 0;

    logic [3:0] R_port_base, G_port_base, B_port_base;
    assign {R_port_base, G_port_base, B_port_base} = {
        selected_pixel[15:12], selected_pixel[10:7], selected_pixel[4:1]
    };

    //--------------------------------------------------------------------------
    //  120×120 가이드 박스(Border) 로직
    //--------------------------------------------------------------------------

    //  120×120 Live 영역 가이드 Border
    // localparam int ROI_W = 120;
    // localparam int ROI_H = 120;
    // localparam int X0 = (160 - ROI_W) / 2;
    // localparam int Y0 = 0;
    // logic border_live;
    // always_comb begin
    //     border_live = 0;
    //     if (display_live) begin
    //         if (((x_pixel == X0 || x_pixel == X0 + ROI_W - 1) && (y_pixel >= Y0 && y_pixel < Y0 + ROI_H)) ||
    //             ((y_pixel == Y0 || y_pixel == Y0 + ROI_H - 1) && (x_pixel >= X0 && x_pixel < X0 + ROI_W)))
    //             border_live = 1;
    //     end
    // end

    //  캡처 영역 Pass/Fail 테두리
    logic border_capture;
    always_comb begin
        border_capture = 0;
        if (display_capture) begin
            // snap 영역 가장 외곽
            if (((x_pixel == 160) || (x_pixel == 319)) && (y_pixel < 120) ||
                ((y_pixel == 0)   || (y_pixel == 119)) && (x_pixel >= 160 && x_pixel < 320))
                border_capture = 1;
        end
    end

    //  사용자 얼굴 가이드 영역
    assign guide_person_on = 
    (x_pixel == 69 && y_pixel == 12) || (x_pixel == 70 && y_pixel == 12) || (x_pixel == 71 && y_pixel == 12) || (x_pixel == 72 && y_pixel == 12) || (x_pixel == 73 && y_pixel == 12) || (x_pixel == 74 && y_pixel == 12) || (x_pixel == 75 && y_pixel == 12) || (x_pixel == 76 && y_pixel == 12) ||
    (x_pixel == 77 && y_pixel == 12) || (x_pixel == 78 && y_pixel == 12) || (x_pixel == 79 && y_pixel == 12) || (x_pixel == 80 && y_pixel == 12) || (x_pixel == 81 && y_pixel == 12) || (x_pixel == 82 && y_pixel == 12) || (x_pixel == 83 && y_pixel == 12) || (x_pixel == 84 && y_pixel == 12) ||
    (x_pixel == 85 && y_pixel == 12) || (x_pixel == 86 && y_pixel == 12) || (x_pixel == 87 && y_pixel == 12) || (x_pixel == 88 && y_pixel == 12) || (x_pixel == 89 && y_pixel == 12) || (x_pixel == 90 && y_pixel == 12) || (x_pixel == 68 && y_pixel == 13) || (x_pixel == 69 && y_pixel == 13) ||
    (x_pixel == 70 && y_pixel == 13) || (x_pixel == 71 && y_pixel == 13) || (x_pixel == 72 && y_pixel == 13) || (x_pixel == 73 && y_pixel == 13) || (x_pixel == 74 && y_pixel == 13) || (x_pixel == 75 && y_pixel == 13) || (x_pixel == 76 && y_pixel == 13) || (x_pixel == 77 && y_pixel == 13) ||
    (x_pixel == 78 && y_pixel == 13) || (x_pixel == 79 && y_pixel == 13) || (x_pixel == 80 && y_pixel == 13) || (x_pixel == 81 && y_pixel == 13) || (x_pixel == 82 && y_pixel == 13) || (x_pixel == 83 && y_pixel == 13) || (x_pixel == 84 && y_pixel == 13) || (x_pixel == 85 && y_pixel == 13) ||
    (x_pixel == 86 && y_pixel == 13) || (x_pixel == 87 && y_pixel == 13) || (x_pixel == 88 && y_pixel == 13) || (x_pixel == 89 && y_pixel == 13) || (x_pixel == 90 && y_pixel == 13) || (x_pixel == 91 && y_pixel == 13) || (x_pixel == 63 && y_pixel == 14) || (x_pixel == 64 && y_pixel == 14) ||
    (x_pixel == 65 && y_pixel == 14) || (x_pixel == 66 && y_pixel == 14) || (x_pixel == 67 && y_pixel == 14) || (x_pixel == 68 && y_pixel == 14) || (x_pixel == 69 && y_pixel == 14) || (x_pixel == 70 && y_pixel == 14) || (x_pixel == 71 && y_pixel == 14) || (x_pixel == 72 && y_pixel == 14) ||
    (x_pixel == 73 && y_pixel == 14) || (x_pixel == 74 && y_pixel == 14) || (x_pixel == 75 && y_pixel == 14) || (x_pixel == 76 && y_pixel == 14) || (x_pixel == 77 && y_pixel == 14) || (x_pixel == 78 && y_pixel == 14) || (x_pixel == 79 && y_pixel == 14) || (x_pixel == 80 && y_pixel == 14) ||
    (x_pixel == 81 && y_pixel == 14) || (x_pixel == 82 && y_pixel == 14) || (x_pixel == 83 && y_pixel == 14) || (x_pixel == 84 && y_pixel == 14) || (x_pixel == 85 && y_pixel == 14) || (x_pixel == 86 && y_pixel == 14) || (x_pixel == 87 && y_pixel == 14) || (x_pixel == 88 && y_pixel == 14) ||
    (x_pixel == 89 && y_pixel == 14) || (x_pixel == 90 && y_pixel == 14) || (x_pixel == 91 && y_pixel == 14) || (x_pixel == 92 && y_pixel == 14) || (x_pixel == 93 && y_pixel == 14) || (x_pixel == 94 && y_pixel == 14) || (x_pixel == 95 && y_pixel == 14) || (x_pixel == 96 && y_pixel == 14) ||
    (x_pixel == 97 && y_pixel == 14) || (x_pixel == 60 && y_pixel == 15) || (x_pixel == 61 && y_pixel == 15) || (x_pixel == 62 && y_pixel == 15) || (x_pixel == 63 && y_pixel == 15) || (x_pixel == 64 && y_pixel == 15) || (x_pixel == 65 && y_pixel == 15) || (x_pixel == 66 && y_pixel == 15) ||
    (x_pixel == 67 && y_pixel == 15) || (x_pixel == 68 && y_pixel == 15) || (x_pixel == 69 && y_pixel == 15) || (x_pixel == 70 && y_pixel == 15) || (x_pixel == 90 && y_pixel == 15) || (x_pixel == 91 && y_pixel == 15) || (x_pixel == 92 && y_pixel == 15) || (x_pixel == 93 && y_pixel == 15) ||
    (x_pixel == 94 && y_pixel == 15) || (x_pixel == 95 && y_pixel == 15) || (x_pixel == 96 && y_pixel == 15) || (x_pixel == 97 && y_pixel == 15) || (x_pixel == 98 && y_pixel == 15) || (x_pixel == 99 && y_pixel == 15) || (x_pixel == 100 && y_pixel == 15) || (x_pixel == 59 && y_pixel == 16) ||
    (x_pixel == 60 && y_pixel == 16) || (x_pixel == 61 && y_pixel == 16) || (x_pixel == 62 && y_pixel == 16) || (x_pixel == 63 && y_pixel == 16) || (x_pixel == 64 && y_pixel == 16) || (x_pixel == 96 && y_pixel == 16) || (x_pixel == 97 && y_pixel == 16) || (x_pixel == 98 && y_pixel == 16) ||
    (x_pixel == 99 && y_pixel == 16) || (x_pixel == 100 && y_pixel == 16) || (x_pixel == 101 && y_pixel == 16) || (x_pixel == 56 && y_pixel == 17) || (x_pixel == 57 && y_pixel == 17) || (x_pixel == 58 && y_pixel == 17) || (x_pixel == 59 && y_pixel == 17) || (x_pixel == 60 && y_pixel == 17) ||
    (x_pixel == 61 && y_pixel == 17) || (x_pixel == 62 && y_pixel == 17) || (x_pixel == 63 && y_pixel == 17) || (x_pixel == 97 && y_pixel == 17) || (x_pixel == 98 && y_pixel == 17) || (x_pixel == 99 && y_pixel == 17) || (x_pixel == 100 && y_pixel == 17) || (x_pixel == 101 && y_pixel == 17) ||
    (x_pixel == 102 && y_pixel == 17) || (x_pixel == 103 && y_pixel == 17) || (x_pixel == 104 && y_pixel == 17) || (x_pixel == 54 && y_pixel == 18) || (x_pixel == 55 && y_pixel == 18) || (x_pixel == 56 && y_pixel == 18) || (x_pixel == 57 && y_pixel == 18) || (x_pixel == 58 && y_pixel == 18) ||
    (x_pixel == 59 && y_pixel == 18) || (x_pixel == 99 && y_pixel == 18) || (x_pixel == 100 && y_pixel == 18) || (x_pixel == 101 && y_pixel == 18) || (x_pixel == 102 && y_pixel == 18) || (x_pixel == 103 && y_pixel == 18) || (x_pixel == 104 && y_pixel == 18) || (x_pixel == 105 && y_pixel == 18) ||
    (x_pixel == 106 && y_pixel == 18) || (x_pixel == 53 && y_pixel == 19) || (x_pixel == 54 && y_pixel == 19) || (x_pixel == 55 && y_pixel == 19) || (x_pixel == 56 && y_pixel == 19) || (x_pixel == 102 && y_pixel == 19) || (x_pixel == 103 && y_pixel == 19) || (x_pixel == 104 && y_pixel == 19) ||
    (x_pixel == 105 && y_pixel == 19) || (x_pixel == 106 && y_pixel == 19) || (x_pixel == 107 && y_pixel == 19) || (x_pixel == 50 && y_pixel == 20) || (x_pixel == 51 && y_pixel == 20) || (x_pixel == 52 && y_pixel == 20) || (x_pixel == 53 && y_pixel == 20) || (x_pixel == 54 && y_pixel == 20) ||
    (x_pixel == 55 && y_pixel == 20) || (x_pixel == 103 && y_pixel == 20) || (x_pixel == 104 && y_pixel == 20) || (x_pixel == 105 && y_pixel == 20) || (x_pixel == 106 && y_pixel == 20) || (x_pixel == 107 && y_pixel == 20) || (x_pixel == 108 && y_pixel == 20) || (x_pixel == 109 && y_pixel == 20) ||
    (x_pixel == 49 && y_pixel == 21) || (x_pixel == 50 && y_pixel == 21) || (x_pixel == 51 && y_pixel == 21) || (x_pixel == 52 && y_pixel == 21) || (x_pixel == 53 && y_pixel == 21) || (x_pixel == 54 && y_pixel == 21) || (x_pixel == 55 && y_pixel == 21) || (x_pixel == 105 && y_pixel == 21) ||
    (x_pixel == 106 && y_pixel == 21) || (x_pixel == 107 && y_pixel == 21) || (x_pixel == 108 && y_pixel == 21) || (x_pixel == 109 && y_pixel == 21) || (x_pixel == 110 && y_pixel == 21) || (x_pixel == 111 && y_pixel == 21) || (x_pixel == 48 && y_pixel == 22) || (x_pixel == 49 && y_pixel == 22) ||
    (x_pixel == 50 && y_pixel == 22) || (x_pixel == 51 && y_pixel == 22) || (x_pixel == 52 && y_pixel == 22) || (x_pixel == 107 && y_pixel == 22) || (x_pixel == 108 && y_pixel == 22) || (x_pixel == 109 && y_pixel == 22) || (x_pixel == 110 && y_pixel == 22) || (x_pixel == 111 && y_pixel == 22) ||
    (x_pixel == 112 && y_pixel == 22) || (x_pixel == 47 && y_pixel == 23) || (x_pixel == 48 && y_pixel == 23) || (x_pixel == 49 && y_pixel == 23) || (x_pixel == 50 && y_pixel == 23) || (x_pixel == 51 && y_pixel == 23) || (x_pixel == 108 && y_pixel == 23) || (x_pixel == 109 && y_pixel == 23) ||
    (x_pixel == 110 && y_pixel == 23) || (x_pixel == 111 && y_pixel == 23) || (x_pixel == 112 && y_pixel == 23) || (x_pixel == 113 && y_pixel == 23) || (x_pixel == 46 && y_pixel == 24) || (x_pixel == 47 && y_pixel == 24) || (x_pixel == 48 && y_pixel == 24) || (x_pixel == 49 && y_pixel == 24) ||
    (x_pixel == 50 && y_pixel == 24) || (x_pixel == 108 && y_pixel == 24) || (x_pixel == 109 && y_pixel == 24) || (x_pixel == 110 && y_pixel == 24) || (x_pixel == 111 && y_pixel == 24) || (x_pixel == 112 && y_pixel == 24) || (x_pixel == 113 && y_pixel == 24) || (x_pixel == 114 && y_pixel == 24) ||
    (x_pixel == 45 && y_pixel == 25) || (x_pixel == 46 && y_pixel == 25) || (x_pixel == 47 && y_pixel == 25) || (x_pixel == 48 && y_pixel == 25) || (x_pixel == 49 && y_pixel == 25) || (x_pixel == 110 && y_pixel == 25) || (x_pixel == 111 && y_pixel == 25) || (x_pixel == 112 && y_pixel == 25) ||
    (x_pixel == 113 && y_pixel == 25) || (x_pixel == 114 && y_pixel == 25) || (x_pixel == 115 && y_pixel == 25) || (x_pixel == 44 && y_pixel == 26) || (x_pixel == 45 && y_pixel == 26) || (x_pixel == 46 && y_pixel == 26) || (x_pixel == 47 && y_pixel == 26) || (x_pixel == 48 && y_pixel == 26) ||
    (x_pixel == 111 && y_pixel == 26) || (x_pixel == 112 && y_pixel == 26) || (x_pixel == 113 && y_pixel == 26) || (x_pixel == 114 && y_pixel == 26) || (x_pixel == 115 && y_pixel == 26) || (x_pixel == 116 && y_pixel == 26) || (x_pixel == 43 && y_pixel == 27) || (x_pixel == 44 && y_pixel == 27) ||
    (x_pixel == 45 && y_pixel == 27) || (x_pixel == 46 && y_pixel == 27) || (x_pixel == 47 && y_pixel == 27) || (x_pixel == 111 && y_pixel == 27) || (x_pixel == 112 && y_pixel == 27) || (x_pixel == 113 && y_pixel == 27) || (x_pixel == 114 && y_pixel == 27) || (x_pixel == 115 && y_pixel == 27) ||
    (x_pixel == 116 && y_pixel == 27) || (x_pixel == 117 && y_pixel == 27) || (x_pixel == 43 && y_pixel == 28) || (x_pixel == 44 && y_pixel == 28) || (x_pixel == 45 && y_pixel == 28) || (x_pixel == 46 && y_pixel == 28) || (x_pixel == 113 && y_pixel == 28) || (x_pixel == 114 && y_pixel == 28) ||
    (x_pixel == 115 && y_pixel == 28) || (x_pixel == 116 && y_pixel == 28) || (x_pixel == 117 && y_pixel == 28) || (x_pixel == 42 && y_pixel == 29) || (x_pixel == 43 && y_pixel == 29) || (x_pixel == 44 && y_pixel == 29) || (x_pixel == 45 && y_pixel == 29) || (x_pixel == 46 && y_pixel == 29) ||
    (x_pixel == 114 && y_pixel == 29) || (x_pixel == 115 && y_pixel == 29) || (x_pixel == 116 && y_pixel == 29) || (x_pixel == 117 && y_pixel == 29) || (x_pixel == 118 && y_pixel == 29) || (x_pixel == 42 && y_pixel == 30) || (x_pixel == 43 && y_pixel == 30) || (x_pixel == 44 && y_pixel == 30) ||
    (x_pixel == 45 && y_pixel == 30) || (x_pixel == 46 && y_pixel == 30) || (x_pixel == 114 && y_pixel == 30) || (x_pixel == 115 && y_pixel == 30) || (x_pixel == 116 && y_pixel == 30) || (x_pixel == 117 && y_pixel == 30) || (x_pixel == 118 && y_pixel == 30) || (x_pixel == 41 && y_pixel == 31) ||
    (x_pixel == 42 && y_pixel == 31) || (x_pixel == 43 && y_pixel == 31) || (x_pixel == 44 && y_pixel == 31) || (x_pixel == 45 && y_pixel == 31) || (x_pixel == 115 && y_pixel == 31) || (x_pixel == 116 && y_pixel == 31) || (x_pixel == 117 && y_pixel == 31) || (x_pixel == 118 && y_pixel == 31) ||
    (x_pixel == 40 && y_pixel == 32) || (x_pixel == 41 && y_pixel == 32) || (x_pixel == 42 && y_pixel == 32) || (x_pixel == 43 && y_pixel == 32) || (x_pixel == 44 && y_pixel == 32) || (x_pixel == 45 && y_pixel == 32) || (x_pixel == 115 && y_pixel == 32) || (x_pixel == 116 && y_pixel == 32) ||
    (x_pixel == 117 && y_pixel == 32) || (x_pixel == 118 && y_pixel == 32) || (x_pixel == 119 && y_pixel == 32) || (x_pixel == 40 && y_pixel == 33) || (x_pixel == 41 && y_pixel == 33) || (x_pixel == 42 && y_pixel == 33) || (x_pixel == 43 && y_pixel == 33) || (x_pixel == 44 && y_pixel == 33) ||
    (x_pixel == 115 && y_pixel == 33) || (x_pixel == 116 && y_pixel == 33) || (x_pixel == 117 && y_pixel == 33) || (x_pixel == 118 && y_pixel == 33) || (x_pixel == 119 && y_pixel == 33) || (x_pixel == 120 && y_pixel == 33) || (x_pixel == 40 && y_pixel == 34) || (x_pixel == 41 && y_pixel == 34) ||
    (x_pixel == 42 && y_pixel == 34) || (x_pixel == 43 && y_pixel == 34) || (x_pixel == 116 && y_pixel == 34) || (x_pixel == 117 && y_pixel == 34) || (x_pixel == 118 && y_pixel == 34) || (x_pixel == 119 && y_pixel == 34) || (x_pixel == 120 && y_pixel == 34) || (x_pixel == 39 && y_pixel == 35) ||
    (x_pixel == 40 && y_pixel == 35) || (x_pixel == 41 && y_pixel == 35) || (x_pixel == 42 && y_pixel == 35) || (x_pixel == 43 && y_pixel == 35) || (x_pixel == 117 && y_pixel == 35) || (x_pixel == 118 && y_pixel == 35) || (x_pixel == 119 && y_pixel == 35) || (x_pixel == 120 && y_pixel == 35) ||
    (x_pixel == 39 && y_pixel == 36) || (x_pixel == 40 && y_pixel == 36) || (x_pixel == 41 && y_pixel == 36) || (x_pixel == 42 && y_pixel == 36) || (x_pixel == 43 && y_pixel == 36) || (x_pixel == 117 && y_pixel == 36) || (x_pixel == 118 && y_pixel == 36) || (x_pixel == 119 && y_pixel == 36) ||
    (x_pixel == 120 && y_pixel == 36) || (x_pixel == 39 && y_pixel == 37) || (x_pixel == 40 && y_pixel == 37) || (x_pixel == 41 && y_pixel == 37) || (x_pixel == 42 && y_pixel == 37) || (x_pixel == 43 && y_pixel == 37) || (x_pixel == 117 && y_pixel == 37) || (x_pixel == 118 && y_pixel == 37) ||
    (x_pixel == 119 && y_pixel == 37) || (x_pixel == 120 && y_pixel == 37) || (x_pixel == 121 && y_pixel == 37) || (x_pixel == 39 && y_pixel == 38) || (x_pixel == 40 && y_pixel == 38) || (x_pixel == 41 && y_pixel == 38) || (x_pixel == 42 && y_pixel == 38) || (x_pixel == 117 && y_pixel == 38) ||
    (x_pixel == 118 && y_pixel == 38) || (x_pixel == 119 && y_pixel == 38) || (x_pixel == 120 && y_pixel == 38) || (x_pixel == 121 && y_pixel == 38) || (x_pixel == 39 && y_pixel == 39) || (x_pixel == 40 && y_pixel == 39) || (x_pixel == 41 && y_pixel == 39) || (x_pixel == 42 && y_pixel == 39) ||
    (x_pixel == 117 && y_pixel == 39) || (x_pixel == 118 && y_pixel == 39) || (x_pixel == 119 && y_pixel == 39) || (x_pixel == 120 && y_pixel == 39) || (x_pixel == 121 && y_pixel == 39) || (x_pixel == 39 && y_pixel == 40) || (x_pixel == 40 && y_pixel == 40) || (x_pixel == 41 && y_pixel == 40) ||
    (x_pixel == 42 && y_pixel == 40) || (x_pixel == 117 && y_pixel == 40) || (x_pixel == 118 && y_pixel == 40) || (x_pixel == 119 && y_pixel == 40) || (x_pixel == 120 && y_pixel == 40) || (x_pixel == 121 && y_pixel == 40) || (x_pixel == 39 && y_pixel == 41) || (x_pixel == 40 && y_pixel == 41) ||
    (x_pixel == 41 && y_pixel == 41) || (x_pixel == 42 && y_pixel == 41) || (x_pixel == 118 && y_pixel == 41) || (x_pixel == 119 && y_pixel == 41) || (x_pixel == 120 && y_pixel == 41) || (x_pixel == 121 && y_pixel == 41) || (x_pixel == 39 && y_pixel == 42) || (x_pixel == 40 && y_pixel == 42) ||
    (x_pixel == 41 && y_pixel == 42) || (x_pixel == 42 && y_pixel == 42) || (x_pixel == 118 && y_pixel == 42) || (x_pixel == 119 && y_pixel == 42) || (x_pixel == 120 && y_pixel == 42) || (x_pixel == 121 && y_pixel == 42) || (x_pixel == 39 && y_pixel == 43) || (x_pixel == 40 && y_pixel == 43) ||
    (x_pixel == 41 && y_pixel == 43) || (x_pixel == 42 && y_pixel == 43) || (x_pixel == 118 && y_pixel == 43) || (x_pixel == 119 && y_pixel == 43) || (x_pixel == 120 && y_pixel == 43) || (x_pixel == 121 && y_pixel == 43) || (x_pixel == 39 && y_pixel == 44) || (x_pixel == 40 && y_pixel == 44) ||
    (x_pixel == 41 && y_pixel == 44) || (x_pixel == 42 && y_pixel == 44) || (x_pixel == 118 && y_pixel == 44) || (x_pixel == 119 && y_pixel == 44) || (x_pixel == 120 && y_pixel == 44) || (x_pixel == 121 && y_pixel == 44) || (x_pixel == 39 && y_pixel == 45) || (x_pixel == 40 && y_pixel == 45) ||
    (x_pixel == 41 && y_pixel == 45) || (x_pixel == 42 && y_pixel == 45) || (x_pixel == 118 && y_pixel == 45) || (x_pixel == 119 && y_pixel == 45) || (x_pixel == 120 && y_pixel == 45) || (x_pixel == 121 && y_pixel == 45) || (x_pixel == 39 && y_pixel == 46) || (x_pixel == 40 && y_pixel == 46) ||
    (x_pixel == 41 && y_pixel == 46) || (x_pixel == 42 && y_pixel == 46) || (x_pixel == 118 && y_pixel == 46) || (x_pixel == 119 && y_pixel == 46) || (x_pixel == 120 && y_pixel == 46) || (x_pixel == 121 && y_pixel == 46) || (x_pixel == 39 && y_pixel == 47) || (x_pixel == 40 && y_pixel == 47) ||
    (x_pixel == 41 && y_pixel == 47) || (x_pixel == 42 && y_pixel == 47) || (x_pixel == 118 && y_pixel == 47) || (x_pixel == 119 && y_pixel == 47) || (x_pixel == 120 && y_pixel == 47) || (x_pixel == 121 && y_pixel == 47) || (x_pixel == 39 && y_pixel == 48) || (x_pixel == 40 && y_pixel == 48) ||
    (x_pixel == 41 && y_pixel == 48) || (x_pixel == 42 && y_pixel == 48) || (x_pixel == 118 && y_pixel == 48) || (x_pixel == 119 && y_pixel == 48) || (x_pixel == 120 && y_pixel == 48) || (x_pixel == 121 && y_pixel == 48) || (x_pixel == 39 && y_pixel == 49) || (x_pixel == 40 && y_pixel == 49) ||
    (x_pixel == 41 && y_pixel == 49) || (x_pixel == 42 && y_pixel == 49) || (x_pixel == 118 && y_pixel == 49) || (x_pixel == 119 && y_pixel == 49) || (x_pixel == 120 && y_pixel == 49) || (x_pixel == 121 && y_pixel == 49) || (x_pixel == 39 && y_pixel == 50) || (x_pixel == 40 && y_pixel == 50) ||
    (x_pixel == 41 && y_pixel == 50) || (x_pixel == 42 && y_pixel == 50) || (x_pixel == 118 && y_pixel == 50) || (x_pixel == 119 && y_pixel == 50) || (x_pixel == 120 && y_pixel == 50) || (x_pixel == 121 && y_pixel == 50) || (x_pixel == 39 && y_pixel == 51) || (x_pixel == 40 && y_pixel == 51) ||
    (x_pixel == 41 && y_pixel == 51) || (x_pixel == 42 && y_pixel == 51) || (x_pixel == 118 && y_pixel == 51) || (x_pixel == 119 && y_pixel == 51) || (x_pixel == 120 && y_pixel == 51) || (x_pixel == 121 && y_pixel == 51) || (x_pixel == 39 && y_pixel == 52) || (x_pixel == 40 && y_pixel == 52) ||
    (x_pixel == 41 && y_pixel == 52) || (x_pixel == 42 && y_pixel == 52) || (x_pixel == 118 && y_pixel == 52) || (x_pixel == 119 && y_pixel == 52) || (x_pixel == 120 && y_pixel == 52) || (x_pixel == 121 && y_pixel == 52) || (x_pixel == 39 && y_pixel == 53) || (x_pixel == 40 && y_pixel == 53) ||
    (x_pixel == 41 && y_pixel == 53) || (x_pixel == 42 && y_pixel == 53) || (x_pixel == 118 && y_pixel == 53) || (x_pixel == 119 && y_pixel == 53) || (x_pixel == 120 && y_pixel == 53) || (x_pixel == 121 && y_pixel == 53) || (x_pixel == 39 && y_pixel == 54) || (x_pixel == 40 && y_pixel == 54) ||
    (x_pixel == 41 && y_pixel == 54) || (x_pixel == 42 && y_pixel == 54) || (x_pixel == 118 && y_pixel == 54) || (x_pixel == 119 && y_pixel == 54) || (x_pixel == 120 && y_pixel == 54) || (x_pixel == 121 && y_pixel == 54) || (x_pixel == 39 && y_pixel == 55) || (x_pixel == 40 && y_pixel == 55) ||
    (x_pixel == 41 && y_pixel == 55) || (x_pixel == 42 && y_pixel == 55) || (x_pixel == 118 && y_pixel == 55) || (x_pixel == 119 && y_pixel == 55) || (x_pixel == 120 && y_pixel == 55) || (x_pixel == 121 && y_pixel == 55) || (x_pixel == 39 && y_pixel == 56) || (x_pixel == 40 && y_pixel == 56) ||
    (x_pixel == 41 && y_pixel == 56) || (x_pixel == 42 && y_pixel == 56) || (x_pixel == 118 && y_pixel == 56) || (x_pixel == 119 && y_pixel == 56) || (x_pixel == 120 && y_pixel == 56) || (x_pixel == 121 && y_pixel == 56) || (x_pixel == 39 && y_pixel == 57) || (x_pixel == 40 && y_pixel == 57) ||
    (x_pixel == 41 && y_pixel == 57) || (x_pixel == 42 && y_pixel == 57) || (x_pixel == 118 && y_pixel == 57) || (x_pixel == 119 && y_pixel == 57) || (x_pixel == 120 && y_pixel == 57) || (x_pixel == 121 && y_pixel == 57) || (x_pixel == 39 && y_pixel == 58) || (x_pixel == 40 && y_pixel == 58) ||
    (x_pixel == 41 && y_pixel == 58) || (x_pixel == 42 && y_pixel == 58) || (x_pixel == 118 && y_pixel == 58) || (x_pixel == 119 && y_pixel == 58) || (x_pixel == 120 && y_pixel == 58) || (x_pixel == 121 && y_pixel == 58) || (x_pixel == 39 && y_pixel == 59) || (x_pixel == 40 && y_pixel == 59) ||
    (x_pixel == 41 && y_pixel == 59) || (x_pixel == 42 && y_pixel == 59) || (x_pixel == 118 && y_pixel == 59) || (x_pixel == 119 && y_pixel == 59) || (x_pixel == 120 && y_pixel == 59) || (x_pixel == 121 && y_pixel == 59) || (x_pixel == 39 && y_pixel == 60) || (x_pixel == 40 && y_pixel == 60) ||
    (x_pixel == 41 && y_pixel == 60) || (x_pixel == 42 && y_pixel == 60) || (x_pixel == 117 && y_pixel == 60) || (x_pixel == 118 && y_pixel == 60) || (x_pixel == 119 && y_pixel == 60) || (x_pixel == 120 && y_pixel == 60) || (x_pixel == 121 && y_pixel == 60) || (x_pixel == 39 && y_pixel == 61) ||
    (x_pixel == 40 && y_pixel == 61) || (x_pixel == 41 && y_pixel == 61) || (x_pixel == 42 && y_pixel == 61) || (x_pixel == 117 && y_pixel == 61) || (x_pixel == 118 && y_pixel == 61) || (x_pixel == 119 && y_pixel == 61) || (x_pixel == 120 && y_pixel == 61) || (x_pixel == 121 && y_pixel == 61) ||
    (x_pixel == 39 && y_pixel == 62) || (x_pixel == 40 && y_pixel == 62) || (x_pixel == 41 && y_pixel == 62) || (x_pixel == 42 && y_pixel == 62) || (x_pixel == 117 && y_pixel == 62) || (x_pixel == 118 && y_pixel == 62) || (x_pixel == 119 && y_pixel == 62) || (x_pixel == 120 && y_pixel == 62) ||
    (x_pixel == 121 && y_pixel == 62) || (x_pixel == 39 && y_pixel == 63) || (x_pixel == 40 && y_pixel == 63) || (x_pixel == 41 && y_pixel == 63) || (x_pixel == 42 && y_pixel == 63) || (x_pixel == 43 && y_pixel == 63) || (x_pixel == 117 && y_pixel == 63) || (x_pixel == 118 && y_pixel == 63) ||
    (x_pixel == 119 && y_pixel == 63) || (x_pixel == 120 && y_pixel == 63) || (x_pixel == 40 && y_pixel == 64) || (x_pixel == 41 && y_pixel == 64) || (x_pixel == 42 && y_pixel == 64) || (x_pixel == 43 && y_pixel == 64) || (x_pixel == 116 && y_pixel == 64) || (x_pixel == 117 && y_pixel == 64) ||
    (x_pixel == 118 && y_pixel == 64) || (x_pixel == 119 && y_pixel == 64) || (x_pixel == 120 && y_pixel == 64) || (x_pixel == 40 && y_pixel == 65) || (x_pixel == 41 && y_pixel == 65) || (x_pixel == 42 && y_pixel == 65) || (x_pixel == 43 && y_pixel == 65) || (x_pixel == 115 && y_pixel == 65) ||
    (x_pixel == 116 && y_pixel == 65) || (x_pixel == 117 && y_pixel == 65) || (x_pixel == 118 && y_pixel == 65) || (x_pixel == 119 && y_pixel == 65) || (x_pixel == 120 && y_pixel == 65) || (x_pixel == 41 && y_pixel == 66) || (x_pixel == 42 && y_pixel == 66) || (x_pixel == 43 && y_pixel == 66) ||
    (x_pixel == 44 && y_pixel == 66) || (x_pixel == 115 && y_pixel == 66) || (x_pixel == 116 && y_pixel == 66) || (x_pixel == 117 && y_pixel == 66) || (x_pixel == 118 && y_pixel == 66) || (x_pixel == 119 && y_pixel == 66) || (x_pixel == 42 && y_pixel == 67) || (x_pixel == 43 && y_pixel == 67) ||
    (x_pixel == 44 && y_pixel == 67) || (x_pixel == 45 && y_pixel == 67) || (x_pixel == 114 && y_pixel == 67) || (x_pixel == 115 && y_pixel == 67) || (x_pixel == 116 && y_pixel == 67) || (x_pixel == 117 && y_pixel == 67) || (x_pixel == 118 && y_pixel == 67) || (x_pixel == 42 && y_pixel == 68) ||
    (x_pixel == 43 && y_pixel == 68) || (x_pixel == 44 && y_pixel == 68) || (x_pixel == 45 && y_pixel == 68) || (x_pixel == 114 && y_pixel == 68) || (x_pixel == 115 && y_pixel == 68) || (x_pixel == 116 && y_pixel == 68) || (x_pixel == 117 && y_pixel == 68) || (x_pixel == 118 && y_pixel == 68) ||
    (x_pixel == 42 && y_pixel == 69) || (x_pixel == 43 && y_pixel == 69) || (x_pixel == 44 && y_pixel == 69) || (x_pixel == 45 && y_pixel == 69) || (x_pixel == 46 && y_pixel == 69) || (x_pixel == 114 && y_pixel == 69) || (x_pixel == 115 && y_pixel == 69) || (x_pixel == 116 && y_pixel == 69) ||
    (x_pixel == 117 && y_pixel == 69) || (x_pixel == 118 && y_pixel == 69) || (x_pixel == 43 && y_pixel == 70) || (x_pixel == 44 && y_pixel == 70) || (x_pixel == 45 && y_pixel == 70) || (x_pixel == 46 && y_pixel == 70) || (x_pixel == 47 && y_pixel == 70) || (x_pixel == 113 && y_pixel == 70) ||
    (x_pixel == 114 && y_pixel == 70) || (x_pixel == 115 && y_pixel == 70) || (x_pixel == 116 && y_pixel == 70) || (x_pixel == 117 && y_pixel == 70) || (x_pixel == 43 && y_pixel == 71) || (x_pixel == 44 && y_pixel == 71) || (x_pixel == 45 && y_pixel == 71) || (x_pixel == 46 && y_pixel == 71) ||
    (x_pixel == 47 && y_pixel == 71) || (x_pixel == 48 && y_pixel == 71) || (x_pixel == 112 && y_pixel == 71) || (x_pixel == 113 && y_pixel == 71) || (x_pixel == 114 && y_pixel == 71) || (x_pixel == 115 && y_pixel == 71) || (x_pixel == 116 && y_pixel == 71) || (x_pixel == 44 && y_pixel == 72) ||
    (x_pixel == 45 && y_pixel == 72) || (x_pixel == 46 && y_pixel == 72) || (x_pixel == 47 && y_pixel == 72) || (x_pixel == 48 && y_pixel == 72) || (x_pixel == 49 && y_pixel == 72) || (x_pixel == 111 && y_pixel == 72) || (x_pixel == 112 && y_pixel == 72) || (x_pixel == 113 && y_pixel == 72) ||
    (x_pixel == 114 && y_pixel == 72) || (x_pixel == 115 && y_pixel == 72) || (x_pixel == 45 && y_pixel == 73) || (x_pixel == 46 && y_pixel == 73) || (x_pixel == 47 && y_pixel == 73) || (x_pixel == 48 && y_pixel == 73) || (x_pixel == 49 && y_pixel == 73) || (x_pixel == 50 && y_pixel == 73) ||
    (x_pixel == 110 && y_pixel == 73) || (x_pixel == 111 && y_pixel == 73) || (x_pixel == 112 && y_pixel == 73) || (x_pixel == 113 && y_pixel == 73) || (x_pixel == 114 && y_pixel == 73) || (x_pixel == 45 && y_pixel == 74) || (x_pixel == 46 && y_pixel == 74) || (x_pixel == 47 && y_pixel == 74) ||
    (x_pixel == 48 && y_pixel == 74) || (x_pixel == 49 && y_pixel == 74) || (x_pixel == 50 && y_pixel == 74) || (x_pixel == 51 && y_pixel == 74) || (x_pixel == 109 && y_pixel == 74) || (x_pixel == 110 && y_pixel == 74) || (x_pixel == 111 && y_pixel == 74) || (x_pixel == 112 && y_pixel == 74) ||
    (x_pixel == 113 && y_pixel == 74) || (x_pixel == 47 && y_pixel == 75) || (x_pixel == 48 && y_pixel == 75) || (x_pixel == 49 && y_pixel == 75) || (x_pixel == 50 && y_pixel == 75) || (x_pixel == 51 && y_pixel == 75) || (x_pixel == 52 && y_pixel == 75) || (x_pixel == 108 && y_pixel == 75) ||
    (x_pixel == 109 && y_pixel == 75) || (x_pixel == 110 && y_pixel == 75) || (x_pixel == 111 && y_pixel == 75) || (x_pixel == 112 && y_pixel == 75) || (x_pixel == 48 && y_pixel == 76) || (x_pixel == 49 && y_pixel == 76) || (x_pixel == 50 && y_pixel == 76) || (x_pixel == 51 && y_pixel == 76) ||
    (x_pixel == 52 && y_pixel == 76) || (x_pixel == 53 && y_pixel == 76) || (x_pixel == 107 && y_pixel == 76) || (x_pixel == 108 && y_pixel == 76) || (x_pixel == 109 && y_pixel == 76) || (x_pixel == 110 && y_pixel == 76) || (x_pixel == 111 && y_pixel == 76) || (x_pixel == 50 && y_pixel == 77) ||
    (x_pixel == 51 && y_pixel == 77) || (x_pixel == 52 && y_pixel == 77) || (x_pixel == 53 && y_pixel == 77) || (x_pixel == 54 && y_pixel == 77) || (x_pixel == 105 && y_pixel == 77) || (x_pixel == 106 && y_pixel == 77) || (x_pixel == 107 && y_pixel == 77) || (x_pixel == 108 && y_pixel == 77) ||
    (x_pixel == 109 && y_pixel == 77) || (x_pixel == 110 && y_pixel == 77) || (x_pixel == 51 && y_pixel == 78) || (x_pixel == 52 && y_pixel == 78) || (x_pixel == 53 && y_pixel == 78) || (x_pixel == 54 && y_pixel == 78) || (x_pixel == 55 && y_pixel == 78) || (x_pixel == 104 && y_pixel == 78) ||
    (x_pixel == 105 && y_pixel == 78) || (x_pixel == 106 && y_pixel == 78) || (x_pixel == 107 && y_pixel == 78) || (x_pixel == 108 && y_pixel == 78) || (x_pixel == 109 && y_pixel == 78) || (x_pixel == 52 && y_pixel == 79) || (x_pixel == 53 && y_pixel == 79) || (x_pixel == 54 && y_pixel == 79) ||
    (x_pixel == 55 && y_pixel == 79) || (x_pixel == 56 && y_pixel == 79) || (x_pixel == 57 && y_pixel == 79) || (x_pixel == 58 && y_pixel == 79) || (x_pixel == 102 && y_pixel == 79) || (x_pixel == 103 && y_pixel == 79) || (x_pixel == 104 && y_pixel == 79) || (x_pixel == 105 && y_pixel == 79) ||
    (x_pixel == 106 && y_pixel == 79) || (x_pixel == 107 && y_pixel == 79) || (x_pixel == 108 && y_pixel == 79) || (x_pixel == 53 && y_pixel == 80) || (x_pixel == 54 && y_pixel == 80) || (x_pixel == 55 && y_pixel == 80) || (x_pixel == 56 && y_pixel == 80) || (x_pixel == 57 && y_pixel == 80) ||
    (x_pixel == 58 && y_pixel == 80) || (x_pixel == 102 && y_pixel == 80) || (x_pixel == 103 && y_pixel == 80) || (x_pixel == 104 && y_pixel == 80) || (x_pixel == 105 && y_pixel == 80) || (x_pixel == 106 && y_pixel == 80) || (x_pixel == 54 && y_pixel == 81) || (x_pixel == 55 && y_pixel == 81) ||
    (x_pixel == 56 && y_pixel == 81) || (x_pixel == 57 && y_pixel == 81) || (x_pixel == 58 && y_pixel == 81) || (x_pixel == 59 && y_pixel == 81) || (x_pixel == 101 && y_pixel == 81) || (x_pixel == 102 && y_pixel == 81) || (x_pixel == 103 && y_pixel == 81) || (x_pixel == 104 && y_pixel == 81) ||
    (x_pixel == 55 && y_pixel == 82) || (x_pixel == 56 && y_pixel == 82) || (x_pixel == 57 && y_pixel == 82) || (x_pixel == 58 && y_pixel == 82) || (x_pixel == 59 && y_pixel == 82) || (x_pixel == 60 && y_pixel == 82) || (x_pixel == 61 && y_pixel == 82) || (x_pixel == 99 && y_pixel == 82) ||
    (x_pixel == 100 && y_pixel == 82) || (x_pixel == 101 && y_pixel == 82) || (x_pixel == 102 && y_pixel == 82) || (x_pixel == 103 && y_pixel == 82) || (x_pixel == 56 && y_pixel == 83) || (x_pixel == 57 && y_pixel == 83) || (x_pixel == 58 && y_pixel == 83) || (x_pixel == 59 && y_pixel == 83) ||
    (x_pixel == 60 && y_pixel == 83) || (x_pixel == 61 && y_pixel == 83) || (x_pixel == 99 && y_pixel == 83) || (x_pixel == 100 && y_pixel == 83) || (x_pixel == 101 && y_pixel == 83) || (x_pixel == 102 && y_pixel == 83) || (x_pixel == 103 && y_pixel == 83) || (x_pixel == 57 && y_pixel == 84) ||
    (x_pixel == 58 && y_pixel == 84) || (x_pixel == 59 && y_pixel == 84) || (x_pixel == 60 && y_pixel == 84) || (x_pixel == 61 && y_pixel == 84) || (x_pixel == 98 && y_pixel == 84) || (x_pixel == 99 && y_pixel == 84) || (x_pixel == 100 && y_pixel == 84) || (x_pixel == 101 && y_pixel == 84) ||
    (x_pixel == 58 && y_pixel == 85) || (x_pixel == 59 && y_pixel == 85) || (x_pixel == 60 && y_pixel == 85) || (x_pixel == 61 && y_pixel == 85) || (x_pixel == 62 && y_pixel == 85) || (x_pixel == 97 && y_pixel == 85) || (x_pixel == 98 && y_pixel == 85) || (x_pixel == 99 && y_pixel == 85) ||
    (x_pixel == 100 && y_pixel == 85) || (x_pixel == 59 && y_pixel == 86) || (x_pixel == 60 && y_pixel == 86) || (x_pixel == 61 && y_pixel == 86) || (x_pixel == 62 && y_pixel == 86) || (x_pixel == 63 && y_pixel == 86) || (x_pixel == 97 && y_pixel == 86) || (x_pixel == 98 && y_pixel == 86) ||
    (x_pixel == 99 && y_pixel == 86) || (x_pixel == 100 && y_pixel == 86) || (x_pixel == 60 && y_pixel == 87) || (x_pixel == 61 && y_pixel == 87) || (x_pixel == 62 && y_pixel == 87) || (x_pixel == 63 && y_pixel == 87) || (x_pixel == 97 && y_pixel == 87) || (x_pixel == 98 && y_pixel == 87) ||
    (x_pixel == 99 && y_pixel == 87) || (x_pixel == 100 && y_pixel == 87) || (x_pixel == 60 && y_pixel == 88) || (x_pixel == 61 && y_pixel == 88) || (x_pixel == 62 && y_pixel == 88) || (x_pixel == 63 && y_pixel == 88) || (x_pixel == 96 && y_pixel == 88) || (x_pixel == 97 && y_pixel == 88) ||
    (x_pixel == 98 && y_pixel == 88) || (x_pixel == 99 && y_pixel == 88) || (x_pixel == 100 && y_pixel == 88) || (x_pixel == 60 && y_pixel == 89) || (x_pixel == 61 && y_pixel == 89) || (x_pixel == 62 && y_pixel == 89) || (x_pixel == 63 && y_pixel == 89) || (x_pixel == 96 && y_pixel == 89) ||
    (x_pixel == 97 && y_pixel == 89) || (x_pixel == 98 && y_pixel == 89) || (x_pixel == 99 && y_pixel == 89) || (x_pixel == 100 && y_pixel == 89) || (x_pixel == 60 && y_pixel == 90) || (x_pixel == 61 && y_pixel == 90) || (x_pixel == 62 && y_pixel == 90) || (x_pixel == 63 && y_pixel == 90) ||
    (x_pixel == 96 && y_pixel == 90) || (x_pixel == 97 && y_pixel == 90) || (x_pixel == 98 && y_pixel == 90) || (x_pixel == 99 && y_pixel == 90) || (x_pixel == 100 && y_pixel == 90) || (x_pixel == 60 && y_pixel == 91) || (x_pixel == 61 && y_pixel == 91) || (x_pixel == 62 && y_pixel == 91) ||
    (x_pixel == 63 && y_pixel == 91) || (x_pixel == 96 && y_pixel == 91) || (x_pixel == 97 && y_pixel == 91) || (x_pixel == 98 && y_pixel == 91) || (x_pixel == 99 && y_pixel == 91) || (x_pixel == 100 && y_pixel == 91) || (x_pixel == 60 && y_pixel == 92) || (x_pixel == 61 && y_pixel == 92) ||
    (x_pixel == 62 && y_pixel == 92) || (x_pixel == 63 && y_pixel == 92) || (x_pixel == 96 && y_pixel == 92) || (x_pixel == 97 && y_pixel == 92) || (x_pixel == 98 && y_pixel == 92) || (x_pixel == 99 && y_pixel == 92) || (x_pixel == 100 && y_pixel == 92) || (x_pixel == 59 && y_pixel == 93) ||
    (x_pixel == 60 && y_pixel == 93) || (x_pixel == 61 && y_pixel == 93) || (x_pixel == 62 && y_pixel == 93) || (x_pixel == 63 && y_pixel == 93) || (x_pixel == 97 && y_pixel == 93) || (x_pixel == 98 && y_pixel == 93) || (x_pixel == 99 && y_pixel == 93) || (x_pixel == 100 && y_pixel == 93) ||
    (x_pixel == 58 && y_pixel == 94) || (x_pixel == 59 && y_pixel == 94) || (x_pixel == 60 && y_pixel == 94) || (x_pixel == 61 && y_pixel == 94) || (x_pixel == 62 && y_pixel == 94) || (x_pixel == 63 && y_pixel == 94) || (x_pixel == 97 && y_pixel == 94) || (x_pixel == 98 && y_pixel == 94) ||
    (x_pixel == 99 && y_pixel == 94) || (x_pixel == 100 && y_pixel == 94) || (x_pixel == 58 && y_pixel == 95) || (x_pixel == 59 && y_pixel == 95) || (x_pixel == 60 && y_pixel == 95) || (x_pixel == 61 && y_pixel == 95) || (x_pixel == 62 && y_pixel == 95) || (x_pixel == 97 && y_pixel == 95) ||
    (x_pixel == 98 && y_pixel == 95) || (x_pixel == 99 && y_pixel == 95) || (x_pixel == 100 && y_pixel == 95) || (x_pixel == 57 && y_pixel == 96) || (x_pixel == 58 && y_pixel == 96) || (x_pixel == 59 && y_pixel == 96) || (x_pixel == 60 && y_pixel == 96) || (x_pixel == 61 && y_pixel == 96) ||
    (x_pixel == 98 && y_pixel == 96) || (x_pixel == 99 && y_pixel == 96) || (x_pixel == 100 && y_pixel == 96) || (x_pixel == 101 && y_pixel == 96) || (x_pixel == 56 && y_pixel == 97) || (x_pixel == 57 && y_pixel == 97) || (x_pixel == 58 && y_pixel == 97) || (x_pixel == 59 && y_pixel == 97) ||
    (x_pixel == 60 && y_pixel == 97) || (x_pixel == 61 && y_pixel == 97) || (x_pixel == 99 && y_pixel == 97) || (x_pixel == 100 && y_pixel == 97) || (x_pixel == 101 && y_pixel == 97) || (x_pixel == 102 && y_pixel == 97) || (x_pixel == 103 && y_pixel == 97) || (x_pixel == 55 && y_pixel == 98) ||
    (x_pixel == 56 && y_pixel == 98) || (x_pixel == 57 && y_pixel == 98) || (x_pixel == 58 && y_pixel == 98) || (x_pixel == 59 && y_pixel == 98) || (x_pixel == 60 && y_pixel == 98) || (x_pixel == 61 && y_pixel == 98) || (x_pixel == 99 && y_pixel == 98) || (x_pixel == 100 && y_pixel == 98) ||
    (x_pixel == 101 && y_pixel == 98) || (x_pixel == 102 && y_pixel == 98) || (x_pixel == 103 && y_pixel == 98) || (x_pixel == 54 && y_pixel == 99) || (x_pixel == 55 && y_pixel == 99) || (x_pixel == 56 && y_pixel == 99) || (x_pixel == 57 && y_pixel == 99) || (x_pixel == 58 && y_pixel == 99) ||
    (x_pixel == 59 && y_pixel == 99) || (x_pixel == 101 && y_pixel == 99) || (x_pixel == 102 && y_pixel == 99) || (x_pixel == 103 && y_pixel == 99) || (x_pixel == 104 && y_pixel == 99) || (x_pixel == 53 && y_pixel == 100) || (x_pixel == 54 && y_pixel == 100) || (x_pixel == 55 && y_pixel == 100) ||
    (x_pixel == 56 && y_pixel == 100) || (x_pixel == 57 && y_pixel == 100) || (x_pixel == 58 && y_pixel == 100) || (x_pixel == 102 && y_pixel == 100) || (x_pixel == 103 && y_pixel == 100) || (x_pixel == 104 && y_pixel == 100) || (x_pixel == 105 && y_pixel == 100) || (x_pixel == 106 && y_pixel == 100) ||
    (x_pixel == 107 && y_pixel == 100) || (x_pixel == 49 && y_pixel == 101) || (x_pixel == 50 && y_pixel == 101) || (x_pixel == 51 && y_pixel == 101) || (x_pixel == 52 && y_pixel == 101) || (x_pixel == 53 && y_pixel == 101) || (x_pixel == 54 && y_pixel == 101) || (x_pixel == 55 && y_pixel == 101) ||
    (x_pixel == 56 && y_pixel == 101) || (x_pixel == 104 && y_pixel == 101) || (x_pixel == 105 && y_pixel == 101) || (x_pixel == 106 && y_pixel == 101) || (x_pixel == 107 && y_pixel == 101) || (x_pixel == 108 && y_pixel == 101) || (x_pixel == 109 && y_pixel == 101) || (x_pixel == 48 && y_pixel == 102) ||
    (x_pixel == 49 && y_pixel == 102) || (x_pixel == 50 && y_pixel == 102) || (x_pixel == 51 && y_pixel == 102) || (x_pixel == 52 && y_pixel == 102) || (x_pixel == 53 && y_pixel == 102) || (x_pixel == 105 && y_pixel == 102) || (x_pixel == 106 && y_pixel == 102) || (x_pixel == 107 && y_pixel == 102) ||
    (x_pixel == 108 && y_pixel == 102) || (x_pixel == 109 && y_pixel == 102) || (x_pixel == 110 && y_pixel == 102) || (x_pixel == 45 && y_pixel == 103) || (x_pixel == 46 && y_pixel == 103) || (x_pixel == 47 && y_pixel == 103) || (x_pixel == 48 && y_pixel == 103) || (x_pixel == 49 && y_pixel == 103) ||
    (x_pixel == 50 && y_pixel == 103) || (x_pixel == 51 && y_pixel == 103) || (x_pixel == 52 && y_pixel == 103) || (x_pixel == 106 && y_pixel == 103) || (x_pixel == 107 && y_pixel == 103) || (x_pixel == 108 && y_pixel == 103) || (x_pixel == 109 && y_pixel == 103) || (x_pixel == 110 && y_pixel == 103) ||
    (x_pixel == 111 && y_pixel == 103) || (x_pixel == 112 && y_pixel == 103) || (x_pixel == 113 && y_pixel == 103) || (x_pixel == 114 && y_pixel == 103) || (x_pixel == 115 && y_pixel == 103) || (x_pixel == 42 && y_pixel == 104) || (x_pixel == 43 && y_pixel == 104) || (x_pixel == 44 && y_pixel == 104) ||
    (x_pixel == 45 && y_pixel == 104) || (x_pixel == 46 && y_pixel == 104) || (x_pixel == 47 && y_pixel == 104) || (x_pixel == 48 && y_pixel == 104) || (x_pixel == 49 && y_pixel == 104) || (x_pixel == 50 && y_pixel == 104) || (x_pixel == 110 && y_pixel == 104) || (x_pixel == 111 && y_pixel == 104) ||
    (x_pixel == 112 && y_pixel == 104) || (x_pixel == 113 && y_pixel == 104) || (x_pixel == 114 && y_pixel == 104) || (x_pixel == 115 && y_pixel == 104) || (x_pixel == 116 && y_pixel == 104) || (x_pixel == 117 && y_pixel == 104) || (x_pixel == 41 && y_pixel == 105) || (x_pixel == 42 && y_pixel == 105) ||
    (x_pixel == 43 && y_pixel == 105) || (x_pixel == 44 && y_pixel == 105) || (x_pixel == 45 && y_pixel == 105) || (x_pixel == 46 && y_pixel == 105) || (x_pixel == 113 && y_pixel == 105) || (x_pixel == 114 && y_pixel == 105) || (x_pixel == 115 && y_pixel == 105) || (x_pixel == 116 && y_pixel == 105) ||
    (x_pixel == 117 && y_pixel == 105) || (x_pixel == 118 && y_pixel == 105) || (x_pixel == 36 && y_pixel == 106) || (x_pixel == 37 && y_pixel == 106) || (x_pixel == 38 && y_pixel == 106) || (x_pixel == 39 && y_pixel == 106) || (x_pixel == 40 && y_pixel == 106) || (x_pixel == 41 && y_pixel == 106) ||
    (x_pixel == 42 && y_pixel == 106) || (x_pixel == 43 && y_pixel == 106) || (x_pixel == 44 && y_pixel == 106) || (x_pixel == 45 && y_pixel == 106) || (x_pixel == 114 && y_pixel == 106) || (x_pixel == 115 && y_pixel == 106) || (x_pixel == 116 && y_pixel == 106) || (x_pixel == 117 && y_pixel == 106) ||
    (x_pixel == 118 && y_pixel == 106) || (x_pixel == 119 && y_pixel == 106) || (x_pixel == 120 && y_pixel == 106) || (x_pixel == 121 && y_pixel == 106) || (x_pixel == 122 && y_pixel == 106) || (x_pixel == 33 && y_pixel == 107) || (x_pixel == 34 && y_pixel == 107) || (x_pixel == 35 && y_pixel == 107) ||
    (x_pixel == 36 && y_pixel == 107) || (x_pixel == 37 && y_pixel == 107) || (x_pixel == 38 && y_pixel == 107) || (x_pixel == 39 && y_pixel == 107) || (x_pixel == 40 && y_pixel == 107) || (x_pixel == 41 && y_pixel == 107) || (x_pixel == 42 && y_pixel == 107) || (x_pixel == 43 && y_pixel == 107) ||
    (x_pixel == 117 && y_pixel == 107) || (x_pixel == 118 && y_pixel == 107) || (x_pixel == 119 && y_pixel == 107) || (x_pixel == 120 && y_pixel == 107) || (x_pixel == 121 && y_pixel == 107) || (x_pixel == 122 && y_pixel == 107) || (x_pixel == 123 && y_pixel == 107) || (x_pixel == 124 && y_pixel == 107) ||
    (x_pixel == 125 && y_pixel == 107) || (x_pixel == 126 && y_pixel == 107) || (x_pixel == 32 && y_pixel == 108) || (x_pixel == 33 && y_pixel == 108) || (x_pixel == 34 && y_pixel == 108) || (x_pixel == 35 && y_pixel == 108) || (x_pixel == 36 && y_pixel == 108) || (x_pixel == 37 && y_pixel == 108) ||
    (x_pixel == 38 && y_pixel == 108) || (x_pixel == 122 && y_pixel == 108) || (x_pixel == 123 && y_pixel == 108) || (x_pixel == 124 && y_pixel == 108) || (x_pixel == 125 && y_pixel == 108) || (x_pixel == 126 && y_pixel == 108) || (x_pixel == 127 && y_pixel == 108) || (x_pixel == 29 && y_pixel == 109) ||
    (x_pixel == 30 && y_pixel == 109) || (x_pixel == 31 && y_pixel == 109) || (x_pixel == 32 && y_pixel == 109) || (x_pixel == 33 && y_pixel == 109) || (x_pixel == 34 && y_pixel == 109) || (x_pixel == 35 && y_pixel == 109) || (x_pixel == 36 && y_pixel == 109) || (x_pixel == 37 && y_pixel == 109) ||
    (x_pixel == 123 && y_pixel == 109) || (x_pixel == 124 && y_pixel == 109) || (x_pixel == 125 && y_pixel == 109) || (x_pixel == 126 && y_pixel == 109) || (x_pixel == 127 && y_pixel == 109) || (x_pixel == 128 && y_pixel == 109) || (x_pixel == 129 && y_pixel == 109) || (x_pixel == 130 && y_pixel == 109) ||
    (x_pixel == 131 && y_pixel == 109) || (x_pixel == 25 && y_pixel == 110) || (x_pixel == 26 && y_pixel == 110) || (x_pixel == 27 && y_pixel == 110) || (x_pixel == 28 && y_pixel == 110) || (x_pixel == 29 && y_pixel == 110) || (x_pixel == 30 && y_pixel == 110) || (x_pixel == 31 && y_pixel == 110) ||
    (x_pixel == 32 && y_pixel == 110) || (x_pixel == 33 && y_pixel == 110) || (x_pixel == 34 && y_pixel == 110) || (x_pixel == 126 && y_pixel == 110) || (x_pixel == 127 && y_pixel == 110) || (x_pixel == 128 && y_pixel == 110) || (x_pixel == 129 && y_pixel == 110) || (x_pixel == 130 && y_pixel == 110) ||
    (x_pixel == 131 && y_pixel == 110) || (x_pixel == 132 && y_pixel == 110) || (x_pixel == 133 && y_pixel == 110) || (x_pixel == 24 && y_pixel == 111) || (x_pixel == 25 && y_pixel == 111) || (x_pixel == 26 && y_pixel == 111) || (x_pixel == 27 && y_pixel == 111) || (x_pixel == 28 && y_pixel == 111) ||
    (x_pixel == 29 && y_pixel == 111) || (x_pixel == 129 && y_pixel == 111) || (x_pixel == 130 && y_pixel == 111) || (x_pixel == 131 && y_pixel == 111) || (x_pixel == 132 && y_pixel == 111) || (x_pixel == 133 && y_pixel == 111) || (x_pixel == 134 && y_pixel == 111) || (x_pixel == 21 && y_pixel == 112) ||
    (x_pixel == 22 && y_pixel == 112) || (x_pixel == 23 && y_pixel == 112) || (x_pixel == 24 && y_pixel == 112) || (x_pixel == 25 && y_pixel == 112) || (x_pixel == 26 && y_pixel == 112) || (x_pixel == 27 && y_pixel == 112) || (x_pixel == 28 && y_pixel == 112) || (x_pixel == 130 && y_pixel == 112) ||
    (x_pixel == 131 && y_pixel == 112) || (x_pixel == 132 && y_pixel == 112) || (x_pixel == 133 && y_pixel == 112) || (x_pixel == 134 && y_pixel == 112) || (x_pixel == 135 && y_pixel == 112) || (x_pixel == 136 && y_pixel == 112) || (x_pixel == 137 && y_pixel == 112) || (x_pixel == 138 && y_pixel == 112) ||
    (x_pixel == 139 && y_pixel == 112) || (x_pixel == 18 && y_pixel == 113) || (x_pixel == 19 && y_pixel == 113) || (x_pixel == 20 && y_pixel == 113) || (x_pixel == 21 && y_pixel == 113) || (x_pixel == 22 && y_pixel == 113) || (x_pixel == 23 && y_pixel == 113) || (x_pixel == 24 && y_pixel == 113) ||
    (x_pixel == 25 && y_pixel == 113) || (x_pixel == 134 && y_pixel == 113) || (x_pixel == 135 && y_pixel == 113) || (x_pixel == 136 && y_pixel == 113) || (x_pixel == 137 && y_pixel == 113) || (x_pixel == 138 && y_pixel == 113) || (x_pixel == 139 && y_pixel == 113) || (x_pixel == 140 && y_pixel == 113) ||
    (x_pixel == 141 && y_pixel == 113) || (x_pixel == 17 && y_pixel == 114) || (x_pixel == 18 && y_pixel == 114) || (x_pixel == 19 && y_pixel == 114) || (x_pixel == 20 && y_pixel == 114) || (x_pixel == 21 && y_pixel == 114) || (x_pixel == 22 && y_pixel == 114) || (x_pixel == 137 && y_pixel == 114) ||
    (x_pixel == 138 && y_pixel == 114) || (x_pixel == 139 && y_pixel == 114) || (x_pixel == 140 && y_pixel == 114) || (x_pixel == 141 && y_pixel == 114) || (x_pixel == 142 && y_pixel == 114) || (x_pixel == 15 && y_pixel == 115) || (x_pixel == 16 && y_pixel == 115) || (x_pixel == 17 && y_pixel == 115) ||
    (x_pixel == 18 && y_pixel == 115) || (x_pixel == 19 && y_pixel == 115) || (x_pixel == 20 && y_pixel == 115) || (x_pixel == 21 && y_pixel == 115) || (x_pixel == 138 && y_pixel == 115) || (x_pixel == 139 && y_pixel == 115) || (x_pixel == 140 && y_pixel == 115) || (x_pixel == 141 && y_pixel == 115) ||
    (x_pixel == 142 && y_pixel == 115) || (x_pixel == 143 && y_pixel == 115) || (x_pixel == 144 && y_pixel == 115) || (x_pixel == 145 && y_pixel == 115) || (x_pixel == 13 && y_pixel == 116) || (x_pixel == 14 && y_pixel == 116) || (x_pixel == 15 && y_pixel == 116) || (x_pixel == 16 && y_pixel == 116) ||
    (x_pixel == 17 && y_pixel == 116) || (x_pixel == 18 && y_pixel == 116) || (x_pixel == 19 && y_pixel == 116) || (x_pixel == 140 && y_pixel == 116) || (x_pixel == 141 && y_pixel == 116) || (x_pixel == 142 && y_pixel == 116) || (x_pixel == 143 && y_pixel == 116) || (x_pixel == 144 && y_pixel == 116) ||
    (x_pixel == 145 && y_pixel == 116);

    logic [2:0] font_row, font_col, font_col_title, font_col_alert;
    //assign font_row = y_pixel[2:0];
    assign font_col = x_pixel[2:0];

    logic [2:0] y_pixel_2 = y_pixel - 2;
    logic [2:0] y_pixel_125 = y_pixel - 125;
    logic [2:0] y_pixel_140 = y_pixel - 140;

    always_comb begin
        if (y_pixel >= 2 && y_pixel < 9) begin
            font_row = y_pixel_2;  // REC, FAIL/PASS 영역    
        end else if (y_pixel >= 125 && y_pixel < 132) begin
            font_row = y_pixel_125;  // 주의사항 텍스트 영역
        end else if (y_pixel >= 140 && y_pixel < 147) begin
            font_row = y_pixel_140;  // Title 영역
        end
    end

    //--------------------------------------------------------------------------
    // REC
    //--------------------------------------------------------------------------
    logic [7:0] char_bitmap_rec;
    logic [6:0] char_code_rec;
    logic text_on_rec;

    logic [23:0] blink_counter;
    logic blink_on;

    always_ff @(posedge clk) blink_counter <= blink_counter + 1;

    assign blink_on = blink_counter[23];
    logic draw_text_rec;
    assign draw_text_rec = (y_pixel >= 2 && y_pixel < 9 && x_pixel < 32);  // 상단 8줄에만 텍스트 표시

    always_comb begin
        char_code_rec = 7'd0;
        if (draw_text_rec) begin
            case (x_pixel >> 3)
                0: char_code_rec = 7'd1;  // ●
                1: char_code_rec = "R";
                2: char_code_rec = "E";
                3: char_code_rec = "C";
                default: char_code_rec = 7'd0;
            endcase
        end
    end

    FontROM fontrom_inst (
        .char_code(char_code_rec),
        .row(font_row),
        .row_data(char_bitmap_rec)
    );

    assign text_on_rec = draw_text_rec && blink_on && char_bitmap_rec[7-font_col];

    //--------------------------------------------------------------------------
    // Title: Face Recognition
    //--------------------------------------------------------------------------
    logic [7:0] char_bitmap_title;
    logic [6:0] char_code_title;
    logic text_on_title;

    localparam int TITLE_ROWS = 8;
    localparam int TITLE_LEN = 23;
    localparam int TEXT_WIDTH = TITLE_LEN * 8;  // 184 
    localparam int START_TITLE_X = (320 - TEXT_WIDTH) / 2;  // 68

    logic draw_text_title;
    assign draw_text_title = (y_pixel >= 140 && y_pixel < 147 
                && x_pixel >= START_TITLE_X && x_pixel < START_TITLE_X + TEXT_WIDTH);

    assign font_col_title = (x_pixel - START_TITLE_X) & 3'd7;

    always_comb begin
        char_code_title = 7'd0;
        if (draw_text_title) begin
            case ((x_pixel - START_TITLE_X) >> 3)
                0: char_code_title = "F";
                1: char_code_title = "A";
                2: char_code_title = "C";
                3: char_code_title = "E";
                4: char_code_title = " ";
                5: char_code_title = "R";
                6: char_code_title = "E";
                7: char_code_title = "C";
                8: char_code_title = "O";
                9: char_code_title = "G";
                10: char_code_title = "N";
                11: char_code_title = "I";
                12: char_code_title = "T";
                13: char_code_title = "I";
                14: char_code_title = "O";
                15: char_code_title = "N";
                16: char_code_title = " ";
                17: char_code_title = "S";
                18: char_code_title = "Y";
                19: char_code_title = "S";
                20: char_code_title = "T";
                21: char_code_title = "E";
                22: char_code_title = "M";
                default: char_code_title = 7'd0;
            endcase
        end
    end

    FontROM fontrom_title (
        .char_code(char_code_title),
        .row(font_row),
        .row_data(char_bitmap_title)
    );

    assign text_on_title = draw_text_title && char_bitmap_title[7-font_col_title];

    //--------------------------------------------------------------------------
    // Alert: PLEASE SET YOUR FACE AREA PROPERLY
    //--------------------------------------------------------------------------
    logic [7:0] char_bitmap_alert;
    logic [6:0] char_code_alert;
    logic text_on_alert;

    localparam int ALERT_ROWS = 8;
    localparam int ALERT_LEN = 35;
    localparam int ALERT_WIDTH = ALERT_LEN * 8;  // 280 
    localparam int START_ALERT_X = (320 - ALERT_WIDTH) / 2;  // 20

    logic draw_text_alert;
    assign draw_text_alert = (y_pixel >= 125 && y_pixel < 132 
                && x_pixel >= START_ALERT_X && x_pixel < START_ALERT_X + ALERT_WIDTH);

    assign font_col_alert = (x_pixel - START_ALERT_X) & 3'd7;

    always_comb begin
        char_code_alert = 7'd0;
        if (draw_text_alert) begin
            case ((x_pixel - START_ALERT_X) >> 3)
                // PLEASE
                0: char_code_alert = "P";
                1: char_code_alert = "L";
                2: char_code_alert = "E";
                3: char_code_alert = "A";
                4: char_code_alert = "S";
                5: char_code_alert = "E";
                6: char_code_alert = 7'd32;  // space

                // SET
                7:  char_code_alert = "S";
                8:  char_code_alert = "E";
                9:  char_code_alert = "T";
                10: char_code_alert = 7'd32;

                // YOUR
                11: char_code_alert = "Y";
                12: char_code_alert = "O";
                13: char_code_alert = "U";
                14: char_code_alert = "R";
                15: char_code_alert = 7'd32;

                // FACE
                16: char_code_alert = "F";
                17: char_code_alert = "A";
                18: char_code_alert = "C";
                19: char_code_alert = "E";
                20: char_code_alert = 7'd32;

                // AREA
                21: char_code_alert = "A";
                22: char_code_alert = "R";
                23: char_code_alert = "E";
                24: char_code_alert = "A";
                25: char_code_alert = 7'd32;

                // PROPERLY.
                26: char_code_alert = "P";
                27: char_code_alert = "R";
                28: char_code_alert = "O";
                29: char_code_alert = "P";
                30: char_code_alert = "E";
                31: char_code_alert = "R";
                32: char_code_alert = "L";
                33: char_code_alert = "Y";
                34: char_code_alert = 7'd46;  // ‘.’ (period)

                default: char_code_alert = 7'd0;
            endcase
        end
    end

    FontROM fontrom_alert (
        .char_code(char_code_alert),
        .row(font_row),
        .row_data(char_bitmap_alert)
    );

    assign text_on_alert = draw_text_alert && char_bitmap_alert[7-font_col_alert];

    //--------------------------------------------------------------------------
    // Mode: Pass/Fail
    //--------------------------------------------------------------------------
    logic [7:0] char_bitmap_mode;
    logic [6:0] char_code_mode;
    logic text_on_mode;

    logic draw_text_mode;
    assign draw_text_mode = (y_pixel >= 2 && y_pixel < 9 && x_pixel >= 281 && x_pixel < 312);

    always_comb begin
        char_code_mode = 7'd0;
        if (draw_text_mode) begin
            char_code_mode = 7'd0;
            if (fail_signal) begin
                case ((x_pixel - 281) >> 3)
                    0: char_code_mode = "F";
                    1: char_code_mode = "A";
                    2: char_code_mode = "I";
                    3: char_code_mode = "L";
                    default: char_code_mode = 7'd0;
                endcase
            end else if (pass_signal) begin
                case ((x_pixel - 281) >> 3)
                    0: char_code_mode = "P";
                    1: char_code_mode = "A";
                    2: char_code_mode = "S";
                    3: char_code_mode = "S";
                    default: char_code_mode = 7'd0;
                endcase
            end
        end
    end

    FontROM fontrom_mode (
        .char_code(char_code_mode),
        .row(font_row),
        .row_data(char_bitmap_mode)
    );

    assign text_on_mode = draw_text_mode && char_bitmap_mode[7-font_col];

    assign d_en = display_live || display_capture || text_on_title || text_on_alert;

    // 5) 최종 포트 드라이브
    always_comb begin
        red_port   = 4'h0;
        green_port = 4'h0;
        blue_port  = 4'h0;
        // 1) Capture Border: Pass=Green, Fail=Red
        if (border_capture && pass_signal) begin
            green_port = 4'hF;
        end else if (border_capture && fail_signal) begin
            red_port = 4'hF;
            // 2) ROI 가이드 영역
        end else if (display_live && guide_person_on) begin
            red_port   = 4'hF;
            green_port = 4'hF;
            blue_port  = 4'hF;
            // 3) REC 텍스트
        end else if (display_live && text_on_rec) begin
            red_port = 4'hF;
            // 4) Mode 텍스트 (Pass/Fail)
        end else if (display_capture && text_on_mode) begin
            if (pass_signal) green_port = 4'hF;
            else if (fail_signal) red_port = 4'hF;
            // 5) title 텍스트
        end else if (display_blank && text_on_title) begin
            red_port   = 4'hF;
            green_port = 4'hF;
            blue_port  = 4'hF;
            // 6) 주의사항 텍스트
        end else if (display_blank && text_on_alert) begin
            red_port   = 4'h0;
            green_port = 4'hF;
            blue_port  = 4'hF;
            // 7) Live/Snap 기본 영상
        end else if (display_live || display_capture) begin
            red_port   = R_port_base;
            green_port = G_port_base;
            blue_port  = B_port_base;
        end
    end

endmodule


module FontROM (
    input  logic [6:0] char_code,
    input  logic [2:0] row,
    output logic [7:0] row_data
);
    always_comb begin
        case ({
            char_code, row
        })

            {7'd1, 3'd0} : row_data = 8'b00000000;  // big dot
            {7'd1, 3'd1} : row_data = 8'b00011000;
            {7'd1, 3'd2} : row_data = 8'b00111100;
            {7'd1, 3'd3} : row_data = 8'b00111100;
            {7'd1, 3'd4} : row_data = 8'b00111100;
            {7'd1, 3'd5} : row_data = 8'b00011000;
            {7'd1, 3'd6} : row_data = 8'b00000000;
            {7'd1, 3'd7} : row_data = 8'b00000000;

            {7'd82, 3'd0} : row_data = 8'b01111100;  // 'R'
            {7'd82, 3'd1} : row_data = 8'b01000010;
            {7'd82, 3'd2} : row_data = 8'b01000010;
            {7'd82, 3'd3} : row_data = 8'b01111100;
            {7'd82, 3'd4} : row_data = 8'b01010000;
            {7'd82, 3'd5} : row_data = 8'b01001000;
            {7'd82, 3'd6} : row_data = 8'b01000100;
            {7'd82, 3'd7} : row_data = 8'b00000000;

            {7'd69, 3'd0} : row_data = 8'b01111110;  // 'E'
            {7'd69, 3'd1} : row_data = 8'b01000000;
            {7'd69, 3'd2} : row_data = 8'b01000000;
            {7'd69, 3'd3} : row_data = 8'b01111100;
            {7'd69, 3'd4} : row_data = 8'b01000000;
            {7'd69, 3'd5} : row_data = 8'b01000000;
            {7'd69, 3'd6} : row_data = 8'b01111110;
            {7'd69, 3'd7} : row_data = 8'b00000000;

            {7'd67, 3'd0} : row_data = 8'b00111100;  // 'C'
            {7'd67, 3'd1} : row_data = 8'b01000010;
            {7'd67, 3'd2} : row_data = 8'b01000000;
            {7'd67, 3'd3} : row_data = 8'b01000000;
            {7'd67, 3'd4} : row_data = 8'b01000000;
            {7'd67, 3'd5} : row_data = 8'b01000010;
            {7'd67, 3'd6} : row_data = 8'b00111100;
            {7'd67, 3'd7} : row_data = 8'b00000000;

            {7'd70, 3'd0} : row_data = 8'b01111110;  // 'F' (char_code = 70)
            {7'd70, 3'd1} : row_data = 8'b01000000;
            {7'd70, 3'd2} : row_data = 8'b01000000;
            {7'd70, 3'd3} : row_data = 8'b01111100;
            {7'd70, 3'd4} : row_data = 8'b01000000;
            {7'd70, 3'd5} : row_data = 8'b01000000;
            {7'd70, 3'd6} : row_data = 8'b01000000;
            {7'd70, 3'd7} : row_data = 8'b00000000;

            {7'd65, 3'd0} : row_data = 8'b00111000;  // 'A' (char_code = 65)
            {7'd65, 3'd1} : row_data = 8'b01000100;
            {7'd65, 3'd2} : row_data = 8'b01000100;
            {7'd65, 3'd3} : row_data = 8'b01111100;
            {7'd65, 3'd4} : row_data = 8'b01000100;
            {7'd65, 3'd5} : row_data = 8'b01000100;
            {7'd65, 3'd6} : row_data = 8'b01000100;
            {7'd65, 3'd7} : row_data = 8'b00000000;

            {7'd73, 3'd0} : row_data = 8'b00111000;  // 'I' (char_code = 73)
            {7'd73, 3'd1} : row_data = 8'b00010000;
            {7'd73, 3'd2} : row_data = 8'b00010000;
            {7'd73, 3'd3} : row_data = 8'b00010000;
            {7'd73, 3'd4} : row_data = 8'b00010000;
            {7'd73, 3'd5} : row_data = 8'b00010000;
            {7'd73, 3'd6} : row_data = 8'b00111000;
            {7'd73, 3'd7} : row_data = 8'b00000000;

            {7'd76, 3'd0} : row_data = 8'b01000000;  // 'L' (char_code = 76)
            {7'd76, 3'd1} : row_data = 8'b01000000;
            {7'd76, 3'd2} : row_data = 8'b01000000;
            {7'd76, 3'd3} : row_data = 8'b01000000;
            {7'd76, 3'd4} : row_data = 8'b01000000;
            {7'd76, 3'd5} : row_data = 8'b01000000;
            {7'd76, 3'd6} : row_data = 8'b01111110;
            {7'd76, 3'd7} : row_data = 8'b00000000;

            {7'd80, 3'd0} : row_data = 8'b01111100;  // 'P' (char_code = 80)
            {7'd80, 3'd1} : row_data = 8'b01000010;
            {7'd80, 3'd2} : row_data = 8'b01000010;
            {7'd80, 3'd3} : row_data = 8'b01111100;
            {7'd80, 3'd4} : row_data = 8'b01000000;
            {7'd80, 3'd5} : row_data = 8'b01000000;
            {7'd80, 3'd6} : row_data = 8'b01000000;
            {7'd80, 3'd7} : row_data = 8'b00000000;

            {7'd83, 3'd0} : row_data = 8'b00111100;  // 'S' (char_code = 83)
            {7'd83, 3'd1} : row_data = 8'b01000000;
            {7'd83, 3'd2} : row_data = 8'b01000000;
            {7'd83, 3'd3} : row_data = 8'b00111100;
            {7'd83, 3'd4} : row_data = 8'b00000010;
            {7'd83, 3'd5} : row_data = 8'b00000010;
            {7'd83, 3'd6} : row_data = 8'b01111100;
            {7'd83, 3'd7} : row_data = 8'b00000000;

            {7'd69, 3'd0} : row_data = 8'b01111110;  // 'E' (char_code = 69)
            {7'd69, 3'd1} : row_data = 8'b01000000;
            {7'd69, 3'd2} : row_data = 8'b01000000;
            {7'd69, 3'd3} : row_data = 8'b01111100;
            {7'd69, 3'd4} : row_data = 8'b01000000;
            {7'd69, 3'd5} : row_data = 8'b01000000;
            {7'd69, 3'd6} : row_data = 8'b01111110;
            {7'd69, 3'd7} : row_data = 8'b00000000;

            {7'd86, 3'd0} : row_data = 8'b01000010;  // 'V' (char_code = 86)
            {7'd86, 3'd1} : row_data = 8'b01000010;
            {7'd86, 3'd2} : row_data = 8'b01000010;
            {7'd86, 3'd3} : row_data = 8'b01000010;
            {7'd86, 3'd4} : row_data = 8'b01000010;
            {7'd86, 3'd5} : row_data = 8'b00100100;
            {7'd86, 3'd6} : row_data = 8'b00011000;
            {7'd86, 3'd7} : row_data = 8'b00000000;

            {7'd78, 3'd0} : row_data = 8'b01000010;  // 'N'
            {7'd78, 3'd1} : row_data = 8'b01100010;
            {7'd78, 3'd2} : row_data = 8'b01010010;
            {7'd78, 3'd3} : row_data = 8'b01001010;
            {7'd78, 3'd4} : row_data = 8'b01000110;
            {7'd78, 3'd5} : row_data = 8'b01000010;
            {7'd78, 3'd6} : row_data = 8'b01000010;
            {7'd78, 3'd7} : row_data = 8'b00000000;

            {7'd71, 3'd0} : row_data = 8'b00111100;  // 'G'
            {7'd71, 3'd1} : row_data = 8'b01000010;
            {7'd71, 3'd2} : row_data = 8'b01000000;
            {7'd71, 3'd3} : row_data = 8'b01001110;
            {7'd71, 3'd4} : row_data = 8'b01000010;
            {7'd71, 3'd5} : row_data = 8'b01000010;
            {7'd71, 3'd6} : row_data = 8'b00111100;
            {7'd71, 3'd7} : row_data = 8'b00000000;

            {7'd79, 3'd0} : row_data = 8'b00111100;  // 'O'
            {7'd79, 3'd1} : row_data = 8'b01000010;
            {7'd79, 3'd2} : row_data = 8'b01000010;
            {7'd79, 3'd3} : row_data = 8'b01000010;
            {7'd79, 3'd4} : row_data = 8'b01000010;
            {7'd79, 3'd5} : row_data = 8'b01000010;
            {7'd79, 3'd6} : row_data = 8'b00111100;
            {7'd79, 3'd7} : row_data = 8'b00000000;

            {7'd84, 3'd0} : row_data = 8'b01111110;  // 'T'
            {7'd84, 3'd1} : row_data = 8'b00010000;
            {7'd84, 3'd2} : row_data = 8'b00010000;
            {7'd84, 3'd3} : row_data = 8'b00010000;
            {7'd84, 3'd4} : row_data = 8'b00010000;
            {7'd84, 3'd5} : row_data = 8'b00010000;
            {7'd84, 3'd6} : row_data = 8'b00010000;
            {7'd84, 3'd7} : row_data = 8'b00000000;

            {7'd89, 3'd0} : row_data = 8'b01000010;  // 'Y'
            {7'd89, 3'd1} : row_data = 8'b01000010;
            {7'd89, 3'd2} : row_data = 8'b00100100;
            {7'd89, 3'd3} : row_data = 8'b00011000;
            {7'd89, 3'd4} : row_data = 8'b00010000;
            {7'd89, 3'd5} : row_data = 8'b00010000;
            {7'd89, 3'd6} : row_data = 8'b00010000;
            {7'd89, 3'd7} : row_data = 8'b00000000;

            {7'd77, 3'd0} : row_data = 8'b01000010;  // 'M'
            {7'd77, 3'd1} : row_data = 8'b01100110;
            {7'd77, 3'd2} : row_data = 8'b01011010;
            {7'd77, 3'd3} : row_data = 8'b01000010;
            {7'd77, 3'd4} : row_data = 8'b01000010;
            {7'd77, 3'd5} : row_data = 8'b01000010;
            {7'd77, 3'd6} : row_data = 8'b01000010;
            {7'd77, 3'd7} : row_data = 8'b00000000;

            {7'd85, 3'd0} : row_data = 8'b10000001;  // 'U'
            {7'd85, 3'd1} : row_data = 8'b10000001;
            {7'd85, 3'd2} : row_data = 8'b10000001;
            {7'd85, 3'd3} : row_data = 8'b10000001;
            {7'd85, 3'd4} : row_data = 8'b10000001;
            {7'd85, 3'd5} : row_data = 8'b10000001;
            {7'd85, 3'd6} : row_data = 8'b01111110;
            {7'd85, 3'd7} : row_data = 8'b00000000;

            {7'd46, 3'd0} : row_data = 8'b00000000;  // '.'
            {7'd46, 3'd1} : row_data = 8'b00000000;
            {7'd46, 3'd2} : row_data = 8'b00000000;
            {7'd46, 3'd3} : row_data = 8'b00000000;
            {7'd46, 3'd4} : row_data = 8'b00000000;
            {7'd46, 3'd5} : row_data = 8'b00000000;
            {7'd46, 3'd6} : row_data = 8'b00011000;
            {7'd46, 3'd7} : row_data = 8'b00000000;

            default: row_data = 8'b00000000;
        endcase
    end
endmodule
