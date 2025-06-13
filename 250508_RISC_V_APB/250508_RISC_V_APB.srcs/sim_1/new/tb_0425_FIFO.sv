`timescale 1ns / 1ps

module tb_apb_fifo_0425;

  logic clk = 0;
  always #5 clk = ~clk;            
  logic reset = 1;

  initial begin
    #20 reset = 0;                 
    #1000 $finish;
  end

  logic        transfer, write, ready;
  logic [31:0] addr, wdata, rdata;
  wire PENABLE;
  wire PSEL6;

  //    APB Master instantiation
  //    PRDATA0-5/PREADY0-5는 사용X
  logic [31:0] PRDATA6;
  logic        PREADY6;

  APB_Master U_MASTER (
    .PCLK    (clk),
    .PRESET  (reset),
    .PADDR   (),         
    .PWDATA  (),
    .PWRITE  (),
    .PENABLE (PENABLE),
    .PSEL0   (), .PSEL1(), .PSEL2(), .PSEL3(),
    .PSEL4   (), .PSEL5(),
    .PSEL6   (PSEL6),        
    .PRDATA0 (32'd0), .PRDATA1(32'd0), .PRDATA2(32'd0),
    .PRDATA3 (32'd0), .PRDATA4(32'd0), .PRDATA5(32'd0),
    .PRDATA6 (PRDATA6),
    .PREADY0 (1'b0),  .PREADY1(1'b0), .PREADY2(1'b0),
    .PREADY3 (1'b0),  .PREADY4(1'b0), .PREADY5(1'b0),
    .PREADY6 (PREADY6),
    .transfer(transfer),
    .ready   (ready),
    .addr    (addr),
    .wdata   (wdata),
    .rdata   (rdata),
    .write   (write)
  );

  //    FIFO Peripheral instantiation
  FIFO_Periph U_FIFO_PERIPH (
    .PCLK    (clk),
    .PRESET  (reset),
    .PADDR   (addr[3:0]),
    .PWDATA  (wdata),
    .PWRITE  (write),
    .PENABLE (PENABLE),
    .PSEL    (PSEL6),
    .PRDATA  (PRDATA6),
    .PREADY  (PREADY6)
  );


  initial begin

    transfer = 0; write = 0; addr = 0; wdata = 0;
    #30;

    // 1) enqueue 4번
    do_tr(1, 32'h1000_6004, 8'hA5);
    do_tr(1, 32'h1000_6004, 8'h5A);
    do_tr(1, 32'h1000_6004, 8'h06);
    do_tr(1, 32'h1000_6004, 8'h29);

    // 2) full
    do_tr(0, 32'h1000_6004, 8'h00);

    // 3) dequeue 4번
    do_tr(0, 32'h1000_6008, 8'h00);
    do_tr(0, 32'h1000_6008, 8'h00);
    do_tr(0, 32'h1000_6008, 8'h00);
    do_tr(0, 32'h1000_6008, 8'h00);

    // 4) empty
    do_tr(0, 32'h1000_6008, 8'h00);
    do_tr(0, 32'h1000_6008, 8'h00);
    do_tr(0, 32'h1000_6008, 8'h00);
  end

  task do_tr(input bit is_wr,
             input logic [31:0] full_addr,
             input logic [7:0]  data8);
    begin
      @(posedge clk);
        transfer <= 1;
        write    <= is_wr;
        addr     <= full_addr;
        wdata    <= {24'd0, data8};
      @(posedge clk);
        transfer <= 0;           // Setup → Access
      wait (ready == 1);         // Master.ready 대기
      @(posedge clk);
      if (!is_wr)
        $display("[%0t] READ @%h -> %h", $time, full_addr, rdata[7:0]);
    end
  endtask

endmodule
