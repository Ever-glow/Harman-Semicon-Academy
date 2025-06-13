`timescale 1ns / 1ps

module APB_SlaveIntf_UART (
    // global signals
    input  logic        PCLK,
    input  logic        PRESET,

    // APB Interface Signals
    input  logic [3:0]  PADDR,
    input  logic [31:0] PWDATA,
    input  logic        PWRITE,
    input  logic        PENABLE,
    input  logic        PSEL,
    output logic [31:0] PRDATA, 
    output logic        PREADY,

    // UART RX Interface
    input  logic [7:0]  uart_rx_data,
    input  logic        uart_rx_done,

    // UART TX Interface
    output logic [7:0]  uart_tx_data,
    output logic        uart_tx_start,
    input  logic        uart_tx_done
);

    //--------------------------------------------------------------------------
    // Internal signals for two FIFOs
    //--------------------------------------------------------------------------
    // RX FIFO control
    logic        rx_wr, rx_rd;
    logic        rx_full, rx_empty;
    logic [7:0]  rx_rdata;

    // TX FIFO control
    logic        tx_wr, tx_rd;
    logic        tx_full, tx_empty;
    logic [7:0]  tx_rdata;
    logic [7:0]  tx_wdata;

    //--------------------------------------------------------------------------
    // RX FIFO 인스턴스화
    //--------------------------------------------------------------------------
    fifo_uart RX_FIFO (
        .clk   (PCLK),
        .reset (PRESET),
        .wdata (uart_rx_data),
        .wr_en (rx_wr),
        .full  (rx_full),
        .rdata (rx_rdata),
        .rd_en (rx_rd),
        .empty (rx_empty)
    );

    //--------------------------------------------------------------------------
    // TX FIFO 인스턴스화
    //--------------------------------------------------------------------------
    fifo_uart TX_FIFO (
        .clk   (PCLK),
        .reset (PRESET),
        .wdata (tx_wdata),
        .wr_en (tx_wr),
        .full  (tx_full),
        .rdata (tx_rdata),
        .rd_en (tx_rd),
        .empty (tx_empty)
    );

    //--------------------------------------------------------------------------
    // RX용 wr, rd 신호 생성 (UART → RX_FIFO)
    //--------------------------------------------------------------------------
    assign rx_wr = uart_rx_done & ~rx_full;
    assign rx_rd = !rx_empty && (PSEL && PENABLE && !PWRITE && (PADDR[3:2] == 2'b01));

    //--------------------------------------------------------------------------
    // TX용 wr, rd 신호 생성 (TX_FIFO → UART)
    //--------------------------------------------------------------------------
    assign tx_wr = !tx_full && (PREADY && (PADDR[3:2] == 2'b11));
    assign tx_rd = !tx_empty && uart_tx_done;

    //--------------------------------------------------------------------------
    // APB Slave FSM: 주소별로 RX/TX 상태·데이터 제어
    //   PADDR[3:2] == 00: RX status
    //                 01: RX data
    //                 10: TX status
    //                 11: TX data write
    //--------------------------------------------------------------------------
    always_ff @(posedge PCLK or posedge PRESET) begin
        if (PRESET) begin
            PRDATA    <= 32'd0;
            PREADY    <= 1'b0;
            tx_wdata  <= 8'd0;
        end 
		else begin
            // 기본값
            PRDATA    <= 32'd0;
            PREADY    <= 1'b0;
            if (PSEL & PENABLE) begin
                case (PADDR[3:2])
                    // 00: RX 상태 읽기
                    2'b00: begin 
						PRDATA <= {30'd0, rx_full, rx_empty};
						PREADY <= 1'b1;
					end
                    // 01: RX 데이터 읽기
                    2'b01: begin
                        if (!rx_empty) begin
                            PRDATA <= {24'd0, rx_rdata};
							PREADY <= 1'b1;
                        end
                    end

                    // 10: TX 상태 읽기
                    2'b10: begin 
						PRDATA <= {30'd0, tx_full, tx_empty};
						PREADY <= 1'b1;
					end
                    // 11: TX 데이터 쓰기
                    2'b11: begin
						if (PWRITE) begin							
							if (!tx_full) begin
								tx_wdata <= PWDATA[7:0];
								PREADY <= 1'b1;
							end
						end
						else begin
							if (!tx_empty) begin
                                PRDATA <= {24'd0, tx_rdata};
								PREADY <= 1'b1;
                            end
						end
                    end
                    default: ;
                endcase
            end
        end
    end

    //--------------------------------------------------------------------------
    // 읽어온 TX 데이터를 UART 송신기로 연결
    //--------------------------------------------------------------------------
    assign uart_tx_data  = tx_rdata;
    assign uart_tx_start = ~tx_empty && ~uart_tx_done;

endmodule

module fifo_uart(
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

    fifo_ram_uart U_RAM_uart(
        .clk(clk),
        .wAddr(wr_ptr),
        .wdata(wdata),
        .wr_en(wr_en & ~full),
        .rAddr(rd_ptr),
        .rdata(rdata)
    );

    fifo_control_unit_uart U_FIFO_ControlUnit_uart(.*);

endmodule

module  fifo_ram_uart (
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

module fifo_control_unit_uart (
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