
`timescale 1 ns / 1 ps

	module myip_axi_spi_m_v1_0 #
	(
		// Users to add parameters here

		// User parameters ends
		// Do not modify the parameters beyond this line


		// Parameters of Axi Slave Bus Interface S00_AXI
		parameter integer C_S00_AXI_DATA_WIDTH	= 32,
		parameter integer C_S00_AXI_ADDR_WIDTH	= 4
	)
	(
		// Users to add ports here
		output wire SCLK,
        output wire MOSI,
        input  wire MISO,
		output wire SS,
        input [7:0] sw,
		// input  [15:0] sw,
		// output [15:0] led,
		// User ports ends
		// Do not modify the ports beyond this line


		// Ports of Axi Slave Bus Interface S00_AXI
		input wire  s00_axi_aclk,
		input wire  s00_axi_aresetn,
		input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
		input wire [2 : 0] s00_axi_awprot,
		input wire  s00_axi_awvalid,
		output wire  s00_axi_awready,
		input wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
		input wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
		input wire  s00_axi_wvalid,
		output wire  s00_axi_wready,
		output wire [1 : 0] s00_axi_bresp,
		output wire  s00_axi_bvalid,
		input wire  s00_axi_bready,
		input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
		input wire [2 : 0] s00_axi_arprot,
		input wire  s00_axi_arvalid,
		output wire  s00_axi_arready,
		output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
		output wire [1 : 0] s00_axi_rresp,
		output wire  s00_axi_rvalid,
		input wire  s00_axi_rready
	);
		wire [7:0] cr, sod, sid, sr;
        wire       sw_sel;
        // external signals
        wire       cpol;
        wire       cpha;
        wire       start;
        wire [7:0] tx_data;
        wire [7:0] rx_data;
        wire       done;
        wire       ready;
        wire [7:0]  tx_data_sw = sw;
        wire [7:0]  tx_data_ax = sod;
// Instantiation of Axi Bus Interface S00_AXI
	myip_axi_spi_m_v1_0_S00_AXI # ( 
		.C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
		.C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
	) myip_axi_spi_m_v1_0_S00_AXI_inst (
		.cr(cr),
        .sod(sod),
        .sid(sid),
        .sr(sr),
		.S_AXI_ACLK(s00_axi_aclk),
		.S_AXI_ARESETN(s00_axi_aresetn),
		.S_AXI_AWADDR(s00_axi_awaddr),
		.S_AXI_AWPROT(s00_axi_awprot),
		.S_AXI_AWVALID(s00_axi_awvalid),
		.S_AXI_AWREADY(s00_axi_awready),
		.S_AXI_WDATA(s00_axi_wdata),
		.S_AXI_WSTRB(s00_axi_wstrb),
		.S_AXI_WVALID(s00_axi_wvalid),
		.S_AXI_WREADY(s00_axi_wready),
		.S_AXI_BRESP(s00_axi_bresp),
		.S_AXI_BVALID(s00_axi_bvalid),
		.S_AXI_BREADY(s00_axi_bready),
		.S_AXI_ARADDR(s00_axi_araddr),
		.S_AXI_ARPROT(s00_axi_arprot),
		.S_AXI_ARVALID(s00_axi_arvalid),
		.S_AXI_ARREADY(s00_axi_arready),
		.S_AXI_RDATA(s00_axi_rdata),
		.S_AXI_RRESP(s00_axi_rresp),
		.S_AXI_RVALID(s00_axi_rvalid),
		.S_AXI_RREADY(s00_axi_rready)
	);

	// Add user logic here
		// AXI → SPI Master
        assign sw_sel   = cr[3];
		assign cpol     = cr[2];
		assign cpha     = cr[1];
		assign start    = cr[0];
		//assign tx_data  = sod;
		// SPI Master -> SPI
		assign sid      = rx_data;            
		assign sr       = {6'b0, ready, done};
        assign tx_data = sw_sel ? tx_data_sw : tx_data_ax;

		SPI_Master U_SPI_Master(
    	// global signals
    	.clk(s00_axi_aclk),
    	.reset(~s00_axi_aresetn),
    	// external signals
    	.cpol(cpol),
    	.cpha(cpha),
    	.start(start),
    	.tx_data(tx_data),
    	.rx_data(rx_data),
    	.done(done),
    	.ready(ready),
    	// external port
    	.SCLK(SCLK),
    	.MOSI(MOSI),
    	.MISO(MISO),
		.SS(SS)
	);

	// 	SPI_Slave U_SPI_SLAVE(
	// 	// global signals
	// 	.clk(s00_axi_aclk),
    // 	.reset(~s00_axi_aresetn),
	// 	// SPI signals
	// 	.SCLK(SCLK),
	// 	.MOSI(MOSI),
	// 	.MISO(MISO),
	// 	.SS(SS),
	// 	.sw(sw),
	// 	.led(led)
	// );
	// User logic ends

	endmodule

	`timescale 1ns / 1ps

module SPI_Master (
    // global signals
    input            clk,
    input            reset,
    // external signals
    input            cpol,
    input            cpha,
    input            start,
    input      [7:0] tx_data,
    output     [7:0] rx_data,
    output reg       done,
    output reg       ready,
    // external port
    output           SCLK,
    output           MOSI,
    input            MISO,
    output           SS
    // to axi
);

    localparam IDLE = 0, CP_DELAY = 1, CP0 = 2, CP1 = 3;

    wire r_sclk;

    reg [1:0] state, state_next;
    reg [7:0] temp_tx_data_reg, temp_tx_data_next;
    reg [5:0] sclk_counter_next, sclk_counter_reg;
    reg [2:0] bit_counter_next, bit_counter_reg;
    reg [7:0] temp_rx_data_reg, temp_rx_data_next;
    reg       ss_reg, ss_next;
    reg [2:0] byte_cnt_reg, byte_cnt_next;

    assign MOSI = temp_tx_data_reg[7];
    assign rx_data = temp_rx_data_reg;
    assign SS = ss_reg;
    //assign byte_cnt = byte_cnt_reg;

    assign r_sclk = ((state == CP1) && ~cpha) || 
                    ((state == CP0) && cpha);
    assign SCLK = cpol ? ~r_sclk : r_sclk;
    // sclk는 cpha, cpol 및 상태에 따라 결정

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            state            <= IDLE;
            temp_tx_data_reg <= 0;
            temp_rx_data_reg <= 0;
            sclk_counter_reg <= 0;
            bit_counter_reg  <= 0;
            ss_reg           <= 1;
            byte_cnt_reg     <= 0;
        end else begin
            state            <= state_next;
            temp_tx_data_reg <= temp_tx_data_next;
            temp_rx_data_reg <= temp_rx_data_next;
            sclk_counter_reg <= sclk_counter_next;
            bit_counter_reg  <= bit_counter_next;
            ss_reg           <= ss_next;
            byte_cnt_reg     <= byte_cnt_next;
        end
    end

    always @(*) begin
        state_next        = state;
        ready             = 0;
        done              = 0;
        //r_sclk     = 0;
        temp_tx_data_next = temp_tx_data_reg;
        temp_rx_data_next = temp_rx_data_reg;
        sclk_counter_next = sclk_counter_reg;
        bit_counter_next  = bit_counter_reg;
        ss_next           = ss_reg;
        byte_cnt_next     = byte_cnt_reg;
        case (state)
            IDLE: begin
                temp_tx_data_next = 0;
                done              = 0;
                ready             = 1;
                ss_next           = 1;
                byte_cnt_next     = 0;
                if (start) begin
                    state_next        = cpha ? CP_DELAY : CP0;
                    temp_tx_data_next = tx_data;
                    ready             = 0;
                    sclk_counter_next = 0;
                    bit_counter_next  = 0;
                    ss_next           = 0;
                    byte_cnt_next     = 0;
                end
            end
            CP_DELAY: begin
                ss_next           = 0;
                if (sclk_counter_reg == 49) begin
                    sclk_counter_next = 0;
                    state_next = CP0;
                end else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end
            CP0: begin
                //r_sclk = 0;
                ss_next           = 0;
                if (sclk_counter_reg == 49) begin
                    temp_rx_data_next = {temp_rx_data_reg[6:0], MISO};
                    sclk_counter_next = 0;
                    state_next = CP1;
                end else begin
                    if (bit_counter_reg == 0) temp_tx_data_next = tx_data;
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end
            CP1: begin
                //r_sclk = 1;
                ss_next           = 0;
                if (bit_counter_reg == 7) done = 1;
                if (sclk_counter_reg == 49) begin
                    sclk_counter_next = 0;
                    if (bit_counter_reg == 7) begin
                        //done = 1;
                        bit_counter_next = 0;
                        if (byte_cnt_reg == 4) begin
                            ss_next = 1; // 4byte 보내면 SS 해제
                            byte_cnt_next = 0;
                            state_next = IDLE;
                        end
                        else begin
                            byte_cnt_next = byte_cnt_reg + 1;
                            state_next    = cpha ? CP_DELAY : CP0;
                        end
                        //state_next = IDLE;
                    end
                    else begin
                        temp_tx_data_next = {temp_tx_data_reg[6:0], 1'b0};
                        bit_counter_next = bit_counter_reg + 1;
                        state_next = CP0;
                    end
                end
                else begin
                    sclk_counter_next = sclk_counter_reg + 1;
                end
            end
        endcase
    end

endmodule
