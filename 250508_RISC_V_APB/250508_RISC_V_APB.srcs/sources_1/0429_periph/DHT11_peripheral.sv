`timescale 1ns / 1ps

module DHT11_Periph (
    input  logic        clk,
    input  logic        reset,
    input  logic [ 3:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    inout  wire         dht_io
);

    logic start_read;
    logic [7:0] o_temp_data_Integral, o_temp_data_Decimal;
    logic [7:0] o_humid_data_Integral, o_humid_data_Decimal;
    logic        error;
    logic [31:0] temp_humi_data;

    DHT11 u_DHT11 (
        .clk            (clk),
        .reset          (reset),
        .start_read     (start_read),
        .o_temp_data_Integral(o_temp_data_Integral),
        .o_temp_data_Decimal (o_temp_data_Decimal),
        .o_humid_data_Integral(o_humid_data_Integral),
        .o_humid_data_Decimal(o_humid_data_Decimal),
        .error          (error),
        .dht_io         (dht_io)
    );

    APB_SlaveIntf_DHT u_APB_SlaveIntf_DHT (
        .clk           (clk),
        .reset         (reset),
        .PADDR         (PADDR),
        .PWDATA        (PWDATA),
        .PWRITE        (PWRITE),
        .PENABLE       (PENABLE),
        .PSEL          (PSEL),
        .PRDATA        (PRDATA),
        .PREADY        (PREADY),
        .start_read    (start_read),
        .temp_humi_data({o_humid_data_Integral, o_humid_data_Decimal, o_temp_data_Integral, o_temp_data_Decimal}),
        .error(error)
    );

endmodule

module APB_SlaveIntf_DHT (
    input  logic        clk,
    input  logic        reset,
    input  logic [ 3:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY,
    output logic        start_read,
    input  logic [31:0] temp_humi_data,
    input  logic        error
);
    logic [31:0] slv_reg0;
    logic [31:0] slv_reg1;
    logic [31:0] slv_reg2;
    logic [31:0] slv_reg3;

    assign start_read = slv_reg0[0];

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            slv_reg0 <= 0; // start
            slv_reg1 <= 0; // temp
            slv_reg2 <= 0; // hum
            slv_reg3 <= 0; // error check
            PREADY   <= 0;
        end else begin
            slv_reg1 <= {16'b0, temp_humi_data[15:0]};   // temp
            slv_reg2 <= {16'b0, temp_humi_data[31:16]};  // humid
            slv_reg3 <= {31'b0, error};
            if (PSEL && PENABLE) begin
                PREADY <= 1;
                if (PWRITE) begin
                    if (PADDR[3:2] == 2'd0) begin
                        slv_reg0 <= PWDATA;
                    // slv_reg1 <= PWDATA;
                    // slv_reg2 <= PWDATA;
                    // slv_reg3 <= PWDATA;
                    end
                end 
                else begin
                    case (PADDR[3:2])
                        2'd0: PRDATA <= slv_reg0;
                        2'd1: PRDATA <= slv_reg1; 
                        2'd2: PRDATA <= slv_reg2;
                        2'd3: PRDATA <= slv_reg3;
                        default: PRDATA <= 32'bx;
                    endcase
                end
            end else begin
                PREADY <= 0;
            end
        end
    end
endmodule


module DHT11 (
    input logic clk,
    input logic reset,
    input logic start_read,
    output logic [7:0] o_temp_data_Integral,
    output logic [7:0] o_temp_data_Decimal,
    output logic [7:0] o_humid_data_Integral,
    output logic [7:0] o_humid_data_Decimal,
    output logic error,
    inout wire dht_io
);

    logic tick;
    logic [39:0] raw_data;

    tick_gen_1micro u_tick_gen_1micro (
        .clk  (clk),
        .reset(reset),
        .tick (tick)
    );

    DHT11_controller u_DHT11_controller (
        .tick(tick),
        .clk(clk),
        .reset(reset),
        .btn_start(start_read),
        .o_data(raw_data),
        .dht_io(dht_io)
    );

    final_select u_final_select (
        .clk(clk),
        .reset(reset),
        .i_data(raw_data),
        .o_temp_data_Integral(o_temp_data_Integral),
        .o_temp_data_Decimal(o_temp_data_Decimal),
        .o_humid_data_Integral(o_humid_data_Integral),
        .o_humid_data_Decimal(o_humid_data_Decimal),
        .error(error)
    );

endmodule


module tick_gen_1micro (
    input  logic clk,
    input  logic reset,
    output logic tick
);

    parameter int COUNT_MAX = 100;

    logic [$clog2(COUNT_MAX)-1:0] count;

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            count <= 0;
            tick  <= 0;
        end else if (count == COUNT_MAX - 1) begin
            tick  <= 1;
            count <= 0;
        end else begin
            count <= count + 1;
            tick  <= 0;
        end
    end

endmodule

module final_select (
    input logic clk,
    input logic reset,
    input logic [39:0] i_data,
    output logic [7:0] o_temp_data_Integral,
    output logic [7:0] o_temp_data_Decimal,
    output logic [7:0] o_humid_data_Integral,
    output logic [7:0] o_humid_data_Decimal,
    output logic error
);

    logic [7:0] RH_Integral, RH_Decimal, T_Integral, T_Decimal, Checksum;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            RH_Integral <= 8'd0;
            RH_Decimal <= 8'd0;
            T_Integral <= 8'd0;
            T_Decimal <= 8'd0;
            Checksum <= 8'd0;
            o_temp_data_Integral <= 8'd0;
            o_temp_data_Decimal <= 8'd0;
            o_humid_data_Integral <= 8'd0;
            o_humid_data_Decimal <= 8'd0;
            error <= 1'b0;
        end else begin
            RH_Integral <= i_data[39:32];
            RH_Decimal <= i_data[31:24];
            T_Integral <= i_data[23:16];
            T_Decimal <= i_data[15:8];
            Checksum <= i_data[7:0];

            o_temp_data_Integral <= T_Integral;  // 항상 온도만 출력
            o_temp_data_Decimal <= T_Decimal;
            o_humid_data_Integral <= RH_Integral;
            o_humid_data_Decimal <= RH_Decimal;

            error <= (Checksum == (RH_Integral + RH_Decimal + T_Integral + T_Decimal)) ? 1'b0 : 1'b1;
        end
    end

endmodule


module DHT11_controller (
    input logic tick,
    input logic clk,
    input logic reset,
    input logic btn_start,
    output logic [39:0] o_data,
    inout wire dht_io
);

    typedef enum logic [3:0] {
        IDLE = 4'b0000,
        START = 4'b0001,
        WAIT = 4'b0010,
        SYNC_LOW = 4'b0011,
        SYNC_HIGH = 4'b0100,
        DATA_SYNC_LOW = 4'b0101,
        DATA_CHECK_HIGH = 4'b0110,
        DATA_HIGH_DONE = 4'b0111,
        STOP = 4'b1000
    } state_t;

    parameter int T_START_LOW = 18000;
    parameter int T_WAIT_HIGH = 30;
    parameter int T_RESP_LOW = 8000;
    parameter int T_RESP_HIGH = 8000;
    parameter int T_DATA01_THRESHOLD = 4000;
    parameter int T_END_LOW = 5000;
    parameter int T_TIMEOUT = 2000000;

    state_t state, next_state;
    logic dht_out_reg, dht_out_next, io_oe_reg, io_oe_next;
    logic [$clog2(T_TIMEOUT)-1:0] count_tick_reg, count_tick_next;
    logic [5:0] data_count_reg, data_count_next;
    logic [39:0] final_data_reg, final_data_next;

    assign o_data = final_data_reg;

    logic dht_io_sync1, dht_io_sync2;
    wire dht_io_rising = ~dht_io_sync2 & dht_io_sync1;
    wire dht_io_falling = dht_io_sync2 & ~dht_io_sync1;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            dht_io_sync1 <= 1'b1;
            dht_io_sync2 <= 1'b1;
        end else begin
            dht_io_sync1 <= dht_io;
            dht_io_sync2 <= dht_io_sync1;
        end
    end

    assign dht_io = (io_oe_reg) ? dht_out_reg : 1'bz;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            count_tick_reg <= 0;
            dht_out_reg <= 1'b1;
            io_oe_reg <= 1'b1;
            data_count_reg <= 0;
            final_data_reg <= 0;
        end else begin
            state <= next_state;
            dht_out_reg <= dht_out_next;
            count_tick_reg <= count_tick_next;
            io_oe_reg <= io_oe_next;
            data_count_reg <= data_count_next;
            final_data_reg <= final_data_next;
        end
    end

    always_comb begin
        next_state = state;
        io_oe_next = io_oe_reg;
        dht_out_next = dht_out_reg;
        count_tick_next = count_tick_reg;
        data_count_next = data_count_reg;
        final_data_next = final_data_reg;

        case (state)
            IDLE: begin
                io_oe_next = 1'b1;
                data_count_next = 0;
                if (btn_start) begin
                    next_state = START;
                    dht_out_next = 1'b0;
                    count_tick_next = 0;
                end
            end

            START: begin
                if (tick) begin
                    count_tick_next++;
                    if (count_tick_reg == T_START_LOW - 1) begin
                        next_state = WAIT;
                        dht_out_next = 1'b1;
                        count_tick_next = 0;
                    end
                end
            end

            WAIT: begin
                if (tick) begin
                    count_tick_next++;
                    if (count_tick_reg == T_WAIT_HIGH - 1) begin
                        next_state = SYNC_LOW;
                        io_oe_next = 1'b0;
                        count_tick_next = 0;
                    end
                end
            end

            SYNC_LOW: begin
                count_tick_next++;
                if (dht_io_rising) begin
                    next_state = SYNC_HIGH;
                    count_tick_next = 0;
                end else if (count_tick_reg == T_TIMEOUT - 1) begin
                    next_state = IDLE;
                    ;
                end
            end

            SYNC_HIGH: begin
                count_tick_next++;
                if (dht_io_falling) begin
                    next_state = DATA_SYNC_LOW;
                    count_tick_next = 0;
                    data_count_next = 0;
                end else if (count_tick_reg == T_TIMEOUT - 1) begin
                    next_state = IDLE;
                end
            end

            DATA_SYNC_LOW: begin
                count_tick_next++;
                if (dht_io_rising) begin
                    next_state = DATA_CHECK_HIGH;
                    count_tick_next = 0;
                end else if (count_tick_reg == T_TIMEOUT - 1) begin
                    next_state = IDLE;
                end
            end

            DATA_CHECK_HIGH: begin
                count_tick_next++;
                if (dht_io_falling) begin
                    next_state = DATA_HIGH_DONE;
                end else if (count_tick_reg == T_TIMEOUT - 1) begin
                    next_state = IDLE;
                end
            end

            DATA_HIGH_DONE: begin
                final_data_next[39 - data_count_reg] = (count_tick_reg > T_DATA01_THRESHOLD) ? 1'b1 : 1'b0;
                data_count_next++;
                count_tick_next = 0;

                next_state = (data_count_reg == 39) ? STOP : DATA_SYNC_LOW;
            end

            STOP: begin
                io_oe_next   = 1'b1;
                dht_out_next = 1'b1;
                count_tick_next++;
                if (count_tick_reg == T_END_LOW - 1) begin
                    next_state = IDLE;
                    count_tick_next = 0;
                end
            end

            default: next_state = IDLE;
        endcase
    end

endmodule
