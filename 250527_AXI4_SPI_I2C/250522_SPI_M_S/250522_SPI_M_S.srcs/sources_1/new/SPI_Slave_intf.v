`timescale 1ns / 1ps

module SPI_Slave (
    // global signals
    input  clk,
    input  resetn,
    // SPI signals
    input  SCLK,
    input  MOSI,
    output MISO,
    input  SS,
    input [15:0] sw,
    output [15:0] led
);
    // internal signals
    wire [7:0] si_data;
    wire       si_done;
    wire [7:0] so_data;
    wire       so_start;
    wire       so_done;
    wire       first_byte_done;

    SPI_Slave_Intf U_SPI_Slave_Intf (
        .clk(clk),
        .resetn(resetn),
        .SCLK(SCLK),
        .MOSI(MOSI),
        .MISO(MISO),
        .SS(SS),
        .si_data(si_data),
        .si_done(si_done),
        .first_byte_done(first_byte_done),
        .so_data(so_data),
        .so_start(so_start),
        .so_done(so_done)
    );

    SPI_Slave_Reg U_SPI_Slave_Reg(
        .clk(clk),
        .resetn(resetn),
        .ss_n(SS),
        .si_data(si_data),
        .si_done(si_done),
        .first_byte_done(first_byte_done),
        .so_data(so_data),
        .so_start(so_start),
        .so_done(so_done),
        .sw(sw),
        .led(led)
    );

endmodule

module SPI_Slave_Intf (
    // global signals
    input        clk,
    input        resetn,
    // SPI signals
    input        SCLK,
    input        MOSI,
    output       MISO,
    input        SS,
    // internal signals
    output [7:0] si_data,
    output       si_done,
    output       first_byte_done,
    input  [7:0] so_data,
    input        so_start,
    output       so_done
);

    reg sclk_sync0, sclk_sync1;
    reg ss_sync0, ss_sync1;
    reg [2:0] byte_cnt_reg, byte_cnt_next;

    always @(posedge clk, negedge resetn) begin
        if (!resetn) begin
            sclk_sync0 <= 0;
            sclk_sync1 <= 0;
            ss_sync0 <= 1'b1;
            ss_sync1 <= 1'b1;
            
        end else begin
            sclk_sync0 <= SCLK;
            sclk_sync1 <= sclk_sync0;
            ss_sync0 <= SS;
            ss_sync1 <= ss_sync0;
            
        end
    end

    wire sclk_rising = sclk_sync0 & ~sclk_sync1;
    wire sclk_falling = ~sclk_sync0 & sclk_sync1;
    // rising / falling edge detector

    wire ss_n = ss_sync1;

    // Slave Input Circuit (MOSI)
    localparam SI_IDLE = 0, SI_CMD = 1, SI_PHASE = 2;

    reg [1:0] si_state, si_state_next;

    reg [7:0] si_data_reg, si_data_next;
    reg [2:0] si_bit_cnt_reg, si_bit_cnt_next;
    reg si_done_reg, si_done_next;
    reg first_byte_done_reg, first_byte_done_next;
    reg [7:0] first_byte_data_reg, first_byte_data_next;
 
    assign si_data = si_data_reg;
    assign si_done = si_done_reg;
    assign first_byte_done = first_byte_done_reg;

    always @(posedge clk, negedge resetn) begin
        if (!resetn) begin
            si_state       <= SI_IDLE;
            si_data_reg    <= 0;
            si_bit_cnt_reg <= 0;
            si_done_reg    <= 1'b0;
            byte_cnt_reg <= 0;
            first_byte_done_reg <= 0;
            first_byte_data_reg  <= 0;
        end else begin
            si_state       <= si_state_next;
            si_data_reg    <= si_data_next;
            si_bit_cnt_reg <= si_bit_cnt_next;
            si_done_reg    <= si_done_next;
            byte_cnt_reg <= byte_cnt_next;
            first_byte_done_reg <= first_byte_done_next;
            first_byte_data_reg <= first_byte_data_next; 
        end
    end

    always @(*) begin
        si_state_next   = si_state;
        si_data_next    = si_data_reg;
        si_bit_cnt_next = si_bit_cnt_reg;
        si_done_next    = 1'b0;
        byte_cnt_next   = byte_cnt_reg;
        first_byte_done_next = first_byte_done_reg;
        first_byte_data_next = first_byte_data_reg;
        case (si_state)
            SI_IDLE: begin
                byte_cnt_next = 0;
                si_done_next = 1'b0;
                first_byte_done_next = 0;
                if (!ss_n) begin
                    si_bit_cnt_next = 0;
                    si_state_next   = SI_CMD;
                end
            end
            SI_CMD: begin
                si_done_next = 1'b0;
                first_byte_done_next = 0;
                if (!ss_n) begin
                    if (sclk_rising) begin
                        si_data_next = {si_data_reg[6:0], MOSI};
                        if (si_bit_cnt_reg == 7) begin
                            first_byte_done_next = 1;
                            si_done_next = 1'b1;
                            si_bit_cnt_next = 0;
                            si_state_next = SI_PHASE;
                        end else begin
                            si_bit_cnt_next = si_bit_cnt_reg + 1;
                        end
                    end
                end else begin
                    si_state_next = SI_IDLE;
                end
            end
            SI_PHASE: begin
                si_done_next = 1'b0;
                first_byte_done_next = 1;
                if (!ss_n) begin
                    if (sclk_rising) begin  // sampling 수행
                        si_data_next = {si_data_reg[6:0], MOSI};
                        if (si_bit_cnt_reg == 7) begin
                            si_bit_cnt_next = 0;
                            si_done_next = 1'b1;                 
                        end else begin
                            si_bit_cnt_next = si_bit_cnt_reg + 1;
                        end
                    end
                end else begin
                    si_state_next = SI_IDLE;
                end
            end
        endcase
    end

    // Slave Output Circuit (MISO)
    localparam SO_IDLE = 0, SO_PHASE = 1;

    reg so_state, so_state_next;

    reg [7:0] so_data_reg, so_data_next;
    reg [7:0] so_data_temp;
    reg [2:0] so_bit_cnt_reg, so_bit_cnt_next;
    reg so_done_reg, so_done_next;
    reg so_data_out, so_data_out_next;

    assign so_done = so_done_reg;
    assign MISO = so_data_out;

    always @(posedge clk, negedge resetn) begin
        if (!resetn) begin
            so_state       <= SO_IDLE;
            so_data_reg    <= 0;
            so_bit_cnt_reg <= 0;
            so_done_reg    <= 1'b0;
            so_data_out    <= 0;
            so_data_temp   <= 0;
        end else begin
            so_state       <= so_state_next;
            so_data_reg    <= so_data_next;
            so_bit_cnt_reg <= so_bit_cnt_next;
            so_done_reg    <= so_done_next;
            so_data_out    <= so_data_out_next;
        end
    end

    always @(*) begin
        so_state_next    = so_state;
        so_data_next     = so_data_reg;
        so_bit_cnt_next  = so_bit_cnt_reg;
        so_done_next     = so_done_reg;
        so_data_out_next = so_data_out;
        case (so_state)
            SO_IDLE: begin
                so_done_next = 1'b0;
                if (!ss_n & so_start) begin
                    so_bit_cnt_next = 0;
                    so_state_next   = SO_PHASE;
                    so_data_next = so_data;
                end
            end
            SO_PHASE: begin
                so_done_next = 1'b0;
                if (!ss_n & so_start) begin
                    if (sclk_falling) begin  // shift 수행, MISO 출력
                        so_data_next      = {so_data_reg[6:0], 1'b0};
                        so_data_out_next  = so_data_reg[7];
                        if (so_bit_cnt_reg == 7) begin
                            so_bit_cnt_next = 0;
                            so_done_next    = 1'b1;
                        end else begin
                            so_bit_cnt_next = so_bit_cnt_reg + 1;
                        end
                    end
                end else begin
                    so_state_next = SO_IDLE;
                end
            end

        endcase
    end

endmodule

module SPI_Slave_Reg (
    // global singals
    input            clk,
    input            resetn,
    // internal signals
    input            ss_n,
    input      [7:0] si_data,
    input            si_done,
    input            first_byte_done,
    //input            so_ready,
    output     [7:0] so_data,
    output           so_start,
    input            so_done,
    input [15:0] sw,
    output [15:0] led
);
    localparam IDLE = 0, ADDR_PHASE = 1, WRITE_PHASE = 2, READ_PHASE = 3;

    reg [7:0] slv_reg0, slv_reg1, slv_reg2, slv_reg3;
    reg [7:0] slv_reg0_next, slv_reg1_next, slv_reg2_next, slv_reg3_next;
    reg [1:0] state, state_next;
    reg [1:0] addr_reg, addr_next;
    reg so_start_reg, so_start_next;
    reg [7:0] so_data_reg, so_data_next;

    assign so_start = so_start_reg;
    assign so_data  = so_data_reg;
    assign led = {slv_reg1, slv_reg0};

    always @(posedge clk, negedge resetn) begin
        if (!resetn) begin
            state        <= IDLE;
            addr_reg     <= 0;
            so_start_reg <= 1'b0;
            so_data_reg  <= 0;
            slv_reg0     <= 0;
            slv_reg1     <= 0;
            slv_reg2     <= 0;
            slv_reg3     <= 0;
        end else begin
            state        <= state_next;
            addr_reg     <= addr_next;
            so_start_reg <= so_start_next;
            so_data_reg  <= so_data_next;
            slv_reg0     <= slv_reg0_next;
            slv_reg1     <= slv_reg1_next;
            slv_reg2     <= slv_reg2_next;
            slv_reg3     <= slv_reg3_next;
        end
    end

    always @(*) begin
        state_next    = state;
        addr_next     = addr_reg;
        so_start_next = so_start_reg;
        so_data_next  = so_data_reg;
        slv_reg0_next = slv_reg0;
        slv_reg1_next = slv_reg1;
        slv_reg2_next = slv_reg2;
        slv_reg3_next = slv_reg3;
        case (state)
            IDLE: begin
                so_start_next = 1'b0;
                addr_next  = 0;
                if (!ss_n) begin
                    //addr_next  = 0;
                    state_next = ADDR_PHASE;
                end
            end
            ADDR_PHASE: begin
                if (!ss_n) begin
                    if (si_done) begin
                        addr_next = si_data[1:0];
                        if (first_byte_done) begin
                            if (si_data[7]) begin
                                state_next = WRITE_PHASE;
                            end
                            else begin
                                state_next = READ_PHASE;
                            end
                        end
                    end
                end else begin
                    state_next = IDLE;
                end
            end
            WRITE_PHASE: begin
                if (!ss_n) begin
                    if (si_done && first_byte_done) begin
                        case (addr_reg)
                            2'd0: slv_reg0_next = si_data;
                            2'd1: slv_reg1_next = si_data;
                            2'd2: slv_reg2_next = si_data;
                            2'd3: slv_reg3_next = si_data;
                        endcase
                        if (addr_reg == 2'd3) begin
                            addr_next = 0;
                        end else begin
                            addr_next = addr_reg + 1;
                        end
                    end
                end else begin
                    state_next = IDLE;
                end
            end
            READ_PHASE: begin
                if (!ss_n) begin
                    so_start_next = 1'b1;
                    case (addr_reg)
                        2'd0: so_data_next = slv_reg0;
                        2'd1: so_data_next = slv_reg1;
                        2'd2: so_data_next = slv_reg2;
                        2'd3: so_data_next = slv_reg3;
                    endcase
                    if (so_done) begin
                        so_start_next = 1'b0;
                        if (addr_reg == 2'd3) begin
                            addr_next = 0;
                        end else begin
                            addr_next = addr_reg + 1;
                        end
                    end
                end else begin
                    state_next = IDLE;
                end
            end
        endcase
    end

endmodule
