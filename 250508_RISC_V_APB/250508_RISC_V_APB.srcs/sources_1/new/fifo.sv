`timescale 1ns / 1ps

module FIFO_Periph (
    // global signal
    input  logic        PCLK,
    input  logic        PRESET,
    // APB Interface Signals
    input  logic [ 3:0] PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA,
    output logic        PREADY
    // outport signals
);
    logic [7:0] wdata, rdata;
    logic       wr_en;
    logic       full;
    logic       rd_en;
    logic       empty;

    fifo U_FIFO(
        .clk(PCLK), 
        .reset(PRESET),
        .wdata(wdata),
        .wr_en(wr_en),
        .full(full),
        .rdata(rdata),
        .rd_en(rd_en),
        .empty(empty)
    );

    APB_SlaveIntf_FIFO U_APB_Intf_FIFO(
        .*,
        .wdata(wdata),
        .wr_en(wr_en),
        .full(full),
        .rdata(rdata),
        .rd_en(rd_en),
        .empty(empty) 
);

endmodule

module APB_SlaveIntf_FIFO (
    // global signal
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
    // internal signals
    output logic [7:0] wdata,
    output logic       wr_en,
    input  logic       full,

    input  logic [7:0] rdata,
    output logic       rd_en,
    input  logic       empty 
);
    //logic [31:0] slv_reg0, slv_reg1; //, slv_reg2; //, slv_reg3;

    //logic [7:0]  FWD, FRD;

    typedef enum logic {IDLE, ACCESS} apb_state_e;
    apb_state_e state;

    always_ff @(posedge PCLK, posedge PRESET) begin
        if (PRESET) begin
            state <= IDLE;
            PREADY <= 0;
            PRDATA <= 32'b0;
            wr_en <= 1'b0;
            rd_en <= 1'b0;
            //FWD <= 8'b0;
           // FRD <= 8'b0;
        end else begin
            PREADY <= 1'b0;
            wr_en  <= 1'b0;
            rd_en  <= 1'b0;
            case (state)
                IDLE: begin
                    if (PSEL & ~PENABLE) state <= ACCESS;
                end 
                ACCESS: begin
                    if (PSEL & PENABLE) begin
                        // write
                        if (PWRITE) begin // write
                            if (PADDR[3:2] == 2'b01) begin
                                //FWD <= PWDATA[7:0];
                                if(full == 1'b0) begin
                                    wdata <= PWDATA[7:0]; // full이 아니면 FIFO에 write
                                    wr_en <= 1'b1;
                                end
                            end
                        end
                        else begin // read
                            if (PADDR[3:2] == 2'b00) begin
                                PRDATA <= {30'd0, full, empty}; // status
                            end
                            else if(PADDR[3:2] == 2'b10) begin 
                                //FRD <= rdata;
                                if(empty == 1'b0) begin
                                    PRDATA <= {24'b0, rdata}; 
                                    rd_en <= 1'b1; 
                                end
                            end
                        end
                        PREADY <= 1'b1;
                        state <= IDLE;
                    end
                end  
            endcase
        end
    end

endmodule

`timescale 1ns / 1ps

module fifo(
    input  logic clk, reset,
    // write side
    input  logic [7:0] wdata,
    input  logic       wr_en,
    output logic       full,
    // read side
    output logic [7:0] rdata,
    input  logic       rd_en,
    output logic       empty
);

    logic [1:0] wr_ptr, rd_ptr;

    fifo_ram U_RAM(
        .clk(clk),
        .wAddr(wr_ptr),
        .wdata(wdata),
        .wr_en(wr_en & ~full),
        .rAddr(rd_ptr),
        .rdata(rdata)
    );

    fifo_control_unit U_FIFO_ControlUnit(.*);

endmodule

module  fifo_ram (
    input              clk, 
    input  logic [1:0] wAddr,
    input  logic [7:0] wdata,
    input  logic       wr_en,
    input  logic [1:0] rAddr,
    output logic [7:0] rdata
);

    logic [7:0] mem[0:2**2-1];

    always_ff @( posedge clk ) begin
        if (wr_en) mem[wAddr] <= wdata;
    end

    assign rdata = mem[rAddr];

endmodule

module fifo_control_unit (
    input  logic clk, reset,
    // write side
    output logic [1:0] wr_ptr,
    input  logic       wr_en,
    output logic       full,
    // read side
    output logic [1:0] rd_ptr,
    input  logic       rd_en,
    output logic       empty
);
    // FSM
    localparam READ = 2'b01, WRITE = 2'b10, READ_WRITE = 2'b11;
    logic [1:0] fifo_state;
    logic [1:0] wr_ptr_next, wr_ptr_reg, rd_ptr_next, rd_ptr_reg;
    logic empty_next, empty_reg, full_next, full_reg;

    assign fifo_state = {wr_en, rd_en};
    assign full       = full_reg;
    assign empty      = empty_reg;
    assign wr_ptr     = wr_ptr_reg;
    assign rd_ptr     = rd_ptr_reg;

    always_ff @( posedge clk, posedge reset ) begin
        if (reset) begin
            wr_ptr_reg <= 0;
            rd_ptr_reg <= 0;
            full_reg   <= 1'b0;
            empty_reg  <= 1'b1;
        end
        else begin
            wr_ptr_reg <= wr_ptr_next;
            rd_ptr_reg <= rd_ptr_next;
            full_reg   <= full_next;
            empty_reg  <= empty_next;
        end
    end

    always_comb begin : fifo_comb
        wr_ptr_next = wr_ptr_reg;
        rd_ptr_next = rd_ptr_reg;
        empty_next = empty_reg;
        full_next = full_reg;
        case (fifo_state)
            READ        : begin
                if (empty_reg == 1'b0) begin
                    full_next = 1'b0;
                    rd_ptr_next = rd_ptr_reg + 1;
                    if (rd_ptr_next == wr_ptr_reg) begin//
                        empty_next = 1'b1;
                    end
                end
            end
            WRITE       : begin
                if (full_reg == 1'b0) begin
                    empty_next = 1'b0;
                    wr_ptr_next = wr_ptr_reg + 1;
                    if (wr_ptr_next == rd_ptr_reg) begin
                        full_next = 1'b1;
                    end
                end
            end
            READ_WRITE  : begin
                if (empty_reg == 1'b1) begin
                    wr_ptr_next = wr_ptr_reg + 1;
                    empty_next  = 1'b0;
                end
                else if (full_reg == 1'b1) begin
                    rd_ptr_next = rd_ptr_reg + 1;
                    full_next   = 1'b0;
                end
                else begin
                    wr_ptr_next = wr_ptr_reg + 1;
                    rd_ptr_next = rd_ptr_reg + 1;
                end
            end 
        endcase
    end
endmodule
