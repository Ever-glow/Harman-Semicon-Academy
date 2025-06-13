`timescale 1ns / 1ps

module fifo(
    input clk, reset,
    // write
    input  [7:0] wdata,
    input        wr,
    output       full,
    // read
    output [7:0] rdata,
    input        rd,
    output       empty
    );

    wire [3:0] waddr, raddr;
    wire w_full;

    register_file U_REG(
        .clk(clk),
        .reset(reset),
        .waddr(waddr),
        .wdata(wdata),
        .wr(~full & wr),
        .raddr(raddr),
        .rdata(rdata)
    );

    fifo_control_unit U_FIFO_CU(
        .clk(clk),
        .reset(reset),
        .wr(wr),
        .waddr(waddr),
        .full(full),
        .rd(rd),
        .raddr(raddr),
        .empty(empty)
    );


endmodule

module register_file (
    input clk,
    input reset,
    // write
    input  [3:0] waddr,
    input  [7:0] wdata,
    input        wr,
    // read
    input  [3:0] raddr,
    output [7:0] rdata
);
    
    reg [7:0] mem [0:15];
    integer i;

    // write
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1) begin
                mem[i] <= 8'd0;
            end
        end
        else begin
            if (wr) begin
                mem[waddr] <= wdata;
            end
        end
    end

    // read
    assign rdata = mem[raddr];

endmodule

module fifo_control_unit (
    input clk, reset,
    // write
    input        wr,
    output [3:0] waddr,
    output       full,
    // read
    input        rd,
    output [3:0] raddr,
    output       empty
);
    // 1bit 상태 outout
    reg full_reg, full_next, empty_reg, empty_next;
    // W,R address관리
    reg [3:0] wptr_reg, wptr_next, rptr_reg, rptr_next;

    assign waddr = wptr_reg;
    assign raddr = rptr_reg;
    assign full = full_reg;
    assign empty = empty_reg;

    // state
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            full_reg <= 0;
            empty_reg <= 1;
            wptr_reg <= 0;
            rptr_reg <= 0;
        end
        else begin
            full_reg  <= full_next;
            empty_reg <= empty_next;
            wptr_reg  <= wptr_next;
            rptr_reg  <= rptr_next;
        end
    end

    always @(*) begin
        full_next  = full_reg;
        empty_next = empty_reg;
        wptr_next  = wptr_reg;
        rptr_next  = rptr_reg;
        case ({wr,rd})
            2'b01: begin        // rd == 1, read
                if(empty_reg == 1'b0) begin
                    rptr_next = rptr_reg + 1;
                    full_next = 1'b0;
                    if (rptr_next == wptr_reg) begin
                        empty_next = 1'b1;
                    end
                end
            end 
            2'b10: begin        // wr == 1, write
                if (full_reg == 1'b0) begin
                    wptr_next = wptr_reg + 1;
                    empty_next = 1'b0;
                    if (wptr_next == rptr_reg) begin
                        full_next = 1'b1;
                    end
                end
            end
            2'b11: begin
                if (empty_reg == 1'b1) begin        // empty, write만 수행
                    wptr_next = wptr_reg + 1;
                    empty_next = 1'b0;
                end
                else if (full_reg == 1'b1) begin    // full, read만 수행
                    rptr_next = rptr_reg + 1;
                    full_next = 1'b0;
                end
                else begin
                    wptr_next = wptr_reg + 1;
                    rptr_next = rptr_reg + 1;
                end
            end
        endcase
    end
    


endmodule
