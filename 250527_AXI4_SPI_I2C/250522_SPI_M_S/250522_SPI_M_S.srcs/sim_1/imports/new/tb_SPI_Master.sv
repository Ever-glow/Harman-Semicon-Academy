`timescale 1ns / 1ps

module tb_SPI_Master ();
    logic       clk;
    logic       reset;
    logic       resetn;

    logic       cpol;
    logic       cpha;
    logic       start;
    logic [7:0] tx_data;
    logic [7:0] rx_data;
    logic       done;
    logic       ready;

    logic       SCLK;
    logic       MOSI;
    logic       MISO;
    logic       SS;

    logic [15:0] sw;
    logic [15:0] led;


    SPI_Master dut (.*);
    SPI_Slave slave_dut(
        .resetn(~reset),
        .*
        );

    
    //assign MISO = MOSI;

    always #5 clk = ~clk;


    initial begin
        clk   = 0;
        reset = 1;
        start = 0;
        #10 reset = 0;
        //SS = 1;

        repeat (3) @(posedge clk);

        // spi master가 spi slave에 write하는 과정의 예시
        //-------------------------------------------//
        // address byte 
        //SS = 0;
        
        tx_data = 8'b10000000; start = 1; cpol = 0; cpha = 0; 
        @(posedge clk);
        start = 0;
        wait (done == 1); wait (done == 0);

        // write data byte on 0x00 address
        
        tx_data = 8'h10;
        wait (done == 1); wait (done == 0); 

        // write data byte in 0x01 address
        
        tx_data = 8'h01; 
        wait (done == 1);wait (done == 0); 
        
        // write data byte in 0x02 address
        
        tx_data = 8'h20; 
        wait (done == 1); wait (done == 0); 

        // write data byte in 0x03 address
        
        tx_data = 8'h02;
        wait (done == 1); wait (done == 0); 

        wait(ready == 1);

        tx_data = 8'b00000000; start = 1; cpol = 0; cpha = 0; 
        @(posedge clk);
        start = 0;
        wait (done == 1); wait (done == 0); 

        wait (done == 1); wait (done == 0); 

        wait (done == 1); wait (done == 0); 

        wait (done == 1); wait (done == 0); 

        wait (done == 1); wait (done == 0); 
        
        

        #2000 $finish;
    end

endmodule
