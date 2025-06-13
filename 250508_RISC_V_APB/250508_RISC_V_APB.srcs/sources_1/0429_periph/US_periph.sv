`timescale 1ns / 1ps

module Ultrasonic_Periph (
    input  logic        clk,
    input  logic        reset,
    input  logic [3:0]  PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    output logic        TRIG,
    input  logic        ECHO
);
    logic start;
    logic [15:0] distance;

    // APB 인터페이스
    APB_SlaveIntf_US u_apb (
        .clk(clk), 
        .reset(reset),
        .PADDR(PADDR), 
        .PWDATA(PWDATA), 
        .PWRITE(PWRITE),
        .PENABLE(PENABLE), 
        .PSEL(PSEL),
        .PRDATA(PRDATA), 
        .PREADY(PREADY),
        .trig_start(start), 
        .distance(distance)
    );

    // 울트라소닉 센서 로직
    Ultrasonic u_us (
        .clk(clk), 
        .reset(reset), 
        .echo(ECHO),
        .start(start), 
        .trigger(TRIG), 
        .distance(distance)
    );
endmodule

// APB 슬레이브 인터페이스
module APB_SlaveIntf_US (
    input  logic        clk,
    input  logic        reset,
    input  logic [3:0]  PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    output logic        trig_start,
    input  logic [15:0] distance
);
    logic [31:0] slv_reg0; 
    logic [31:0] slv_reg1;

    assign trig_start = slv_reg0[0];

    always_ff @(posedge clk , posedge reset) begin
        if (reset) begin
            slv_reg0 <= 0;
            slv_reg1 <= 0;
            //slv_reg2 <= 0;
            //slv_reg3 <= 0;
            PREADY   <= 0;
        end else begin
            slv_reg1 <= {16'b0, distance};
            if (PSEL && PENABLE) begin
                PREADY <= 1;
                if (PWRITE) begin
                    if (PADDR[3:2] == 2'd0)
                        slv_reg0 <= PWDATA;
                        //slv_reg1 <= PWDATA;
                        //slv_reg2 <= PWDATA;
                        //slv_reg3 <= PWDATA;
                end else begin
                    case (PADDR[3:2])
                        2'd0: PRDATA <= slv_reg0;
                        2'd1: PRDATA <= slv_reg1;
                        //2'd1: PRDATA <= slv_reg2;
                        //2'd1: PRDATA <= slv_reg3;
                        default: PRDATA <= 32'bx;
                    endcase
                end
            end else begin
                PREADY <= 0;
            end
        end
    end
endmodule


module Ultrasonic (
    input  logic clk,
    input  logic reset,
    input  logic echo,
    input  logic start,
    output logic trigger,
    output logic [15:0] distance
);
    logic clk_1us;
    logic [19:0] time_count;

    clk_divider_1 u_div (
        .clk(clk), 
        .reset(reset), 
        .o_clk(clk_1us)
    );

    us_cu u_fsm (
        .clk(clk_1us), 
        .reset(reset), 
        .echo(echo),
        .start(start), 
        .trigger(trigger), 
        .time_count(time_count)
    );

    dist_calc u_dist (
        .time_count(time_count), 
        .distance(distance)
    );
endmodule

 
module clk_divider_1 (
    input  logic clk,
    input  logic reset,
    output logic o_clk
);
    parameter CLK_DIV = 50;
    logic [$clog2(CLK_DIV)-1:0] count;

    always_ff @(posedge clk , posedge reset) begin
        if (reset) begin
            count <= 0;
            o_clk <= 0;
        end else begin
            if (count == CLK_DIV - 1) begin
                count <= 0;
                o_clk <= ~o_clk;
            end else begin
                count <= count + 1;
            end
        end
    end
endmodule


module us_cu (
    input  logic clk,
    input  logic reset,
    input  logic echo,
    input  logic start,
    output logic trigger,
    output logic [19:0] time_count
);
    logic [19:0] counter_trigger, counter_echo;
    logic echo_prev, measuring, triggered;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            trigger         <= 0;
            counter_trigger <= 0;
            counter_echo    <= 0;
            time_count      <= 0;
            echo_prev       <= 0;
            measuring       <= 0;
            triggered       <= 0;
        end else begin
            echo_prev <= echo;

            // Trigger Pulse 생성 (10us 유지)
            if (start && !triggered) begin
                trigger         <= 1;
                counter_trigger <= 0;
                triggered       <= 1;
            end else if (trigger) begin
                if (counter_trigger < 10)
                    counter_trigger <= counter_trigger + 1;
                else
                    trigger <= 0;
            end

            // Echo 측정
            if (echo && !echo_prev) begin  // 상승 에지에서 시작
                counter_echo <= 0;
                measuring    <= 1;
            end else if (!echo && echo_prev && measuring) begin  // 하강 에지에서 완료
                time_count <= counter_echo;
                measuring  <= 0;
                triggered  <= 0;
            end else if (measuring) begin
                counter_echo <= counter_echo + 1;
            end
        end
    end
endmodule


module dist_calc (
    input  logic [19:0] time_count,
    output logic [15:0] distance
);
    logic [31:0] temp;
    always_comb begin
        temp = time_count * 32'd34;
        distance = temp / 32'd2000;
        if (distance > 16'hFFFF)
            distance = 16'hFFFF;
    end
endmodule