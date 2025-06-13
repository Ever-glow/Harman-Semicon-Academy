`timescale 1ns / 1ps

module frame_uart_sender #(
    // Full frame dimensions
    parameter int FULL_W      = 160,
    parameter int FULL_H      = 120,
    // Region of Interest (central 120x120)
    parameter int ROI_W       = 120,
    parameter int ROI_H       = 120,
    parameter int IMG_WIDTH   = 120,
    localparam int ROI_PIXELS = ROI_W * ROI_H
) (
    input  logic        clk,
    input  logic        reset,           // Active-high asynchronous reset
    input  logic        capture_trigger, // 1-clock pulse to start capture

    // BRAM read port (synchronous)
    output logic        oe_ram,          // read enable
    output logic [14:0] rAddr_ram,       // read address
    input  logic [15:0] rData_ram,       // 16-bit RGB565 data

    // UART TX interface
    output logic [7:0]  uart_data,       // 8-bit data bus
    output logic        uart_start,      // 1-clock start pulse
    input  logic        uart_busy,       // high while TX busy
    input  logic        uart_ready,      // high when TX ready

    // Frame send-done pulse
    output logic        send_done,       // 1-clock pulse on last pixel
    input  logic        sw_gaussian,
    input  logic        sw_filter 
);

    // FSM states
    typedef enum logic [3:0] {
        ST_IDLE     = 4'd0,
        ST_READ     = 4'd1,
        ST_WAIT     = 4'd2,
        ST_CAPTURE  = 4'd3,
        ST_SEND_R   = 4'd4,
        ST_WAIT_R   = 4'd5,
        ST_SEND_G   = 4'd6,
        ST_WAIT_G   = 4'd7,
        ST_SEND_B   = 4'd8,
        ST_WAIT_B   = 4'd9,
        ST_INCR     = 4'd10,
        ST_DONE     = 4'd11,
        ST_LINEBUF  = 4'd12,
        ST_SHARPEN  = 4'd13,
        ST_GAUSSIAN = 4'd14
    } state_t;

    state_t        state, next_state;
    logic [13:0]   addr_cnt;        // up to 14400 pixels (14 bits)
    logic [15:0]   pixel_data;
    logic [7:0]    r8, g8, b8;      // expanded 8-bit channels
    logic [7:0]    red_data,blue_data,green_data;    
    logic [7:0] red_line[0:2][0:IMG_WIDTH-1];
    logic [7:0] green_line[0:2][0:IMG_WIDTH-1];
    logic [7:0] blue_line[0:2][0:IMG_WIDTH-1];
    logic [7:0] sharp_data_red, sharp_data_green, sharp_data_blue;
    logic [7:0] gaussian_data_red,gaussian_data_green, gaussian_data_blue;
    logic [15:0] tmp_gaussian_blue, tmp_gaussian_green, tmp_gaussian_red;
    assign red_data = r8; 
    assign blue_data = b8; 
    assign green_data = g8;    
    

    assign gaussian_data_red = sw_gaussian? tmp_gaussian_red >> 8 : red_data;
    assign gaussian_data_green = sw_gaussian ? tmp_gaussian_green >> 8 : green_data;
    assign gaussian_data_blue = sw_gaussian ?  tmp_gaussian_blue >> 8 : blue_data;

    // logic [7:0] gamma_data;
    // logic [7:0] gamma_lut[0:255];
    // initial $readmemh("gamma_lut.hex", gamma_lut);

    logic [7:0] window_red[0:2][0:2];
    logic [7:0] window_green[0:2][0:2];
    logic [7:0] window_blue[0:2][0:2];
    logic [7:0] col_cnt;
    logic sel_buf; 


    // Asynchronous reset, state + regs update
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state       <= ST_IDLE;
            addr_cnt    <= 0;
            col_cnt <= 0;
            pixel_data  <= 16'd0;
            r8          <= 8'd0;
            g8          <= 8'd0;
            b8          <= 8'd0;
            oe_ram      <= 1'b0;
            rAddr_ram   <= '0;
            uart_data   <= 8'd0;
            uart_start  <= 1'b0;
            send_done   <= 1'b0;
            sel_buf     <= 0;
        end else begin
            // clear one-cycle pulses
            uart_start <= 1'b0;
            send_done  <= 1'b0;
            state      <= next_state;

            case (state)
                ST_IDLE: begin
                    oe_ram <= 1'b0;
                    if (capture_trigger) begin
                        addr_cnt <= 14'd0;
                    end
                end

                ST_READ: begin
                    oe_ram    <= 1'b1;
                    // calculate ROI address offset
                    // center ROI: start_x = (FULL_W - ROI_W)/2, start_y = (FULL_H - ROI_H)/2
                    rAddr_ram <= (( (FULL_H-ROI_H)>>1 ) + (addr_cnt / ROI_W)) * FULL_W  //120 * 120 으로 변경 
                                 + ( (FULL_W-ROI_W)>>1 ) + (addr_cnt % ROI_W);
                end

                ST_WAIT: begin
                    oe_ram <= 1'b0;
                end

                ST_CAPTURE: begin
                    pixel_data <= rData_ram;
                    // expand 5/6/5 bits to 8 bits via replication
                    r8 <= { pixel_data[15:11], pixel_data[15:13] };
                    g8 <= { pixel_data[10:5],  pixel_data[10:9]  };
                    b8 <= { pixel_data[4:0],   pixel_data[4:2]   };
                    col_cnt <= addr_cnt % ROI_W;
                end
                 ST_LINEBUF: begin
                    // Shift in gray data to line buffer
                case(sel_buf)
                    0: begin
                        red_line[0][col_cnt] <= red_line[1][col_cnt];
                        red_line[1][col_cnt] <= red_line[2][col_cnt];
                        red_line[2][col_cnt] <= red_data;

                        green_line[0][col_cnt] <= green_line[1][col_cnt];
                        green_line[1][col_cnt] <= green_line[2][col_cnt];
                        green_line[2][col_cnt] <= green_data;

                        blue_line[0][col_cnt] <= blue_line[1][col_cnt];
                        blue_line[1][col_cnt] <= blue_line[2][col_cnt];
                        blue_line[2][col_cnt] <= blue_data;
                    end
                    1: begin
                        red_line[0][col_cnt] <= red_line[1][col_cnt];
                        red_line[1][col_cnt] <= red_line[2][col_cnt];
                        red_line[2][col_cnt] <= gaussian_data_red;

                        green_line[0][col_cnt] <= green_line[1][col_cnt];
                        green_line[1][col_cnt] <= green_line[2][col_cnt];
                        green_line[2][col_cnt] <= gaussian_data_green;

                        blue_line[0][col_cnt] <= blue_line[1][col_cnt];
                        blue_line[1][col_cnt] <= blue_line[2][col_cnt];
                        blue_line[2][col_cnt] <= gaussian_data_blue;
                    end
                    default: begin
                        red_line[0][col_cnt] <= red_line[1][col_cnt];
                        red_line[1][col_cnt] <= red_line[2][col_cnt];
                        red_line[2][col_cnt] <= red_data;

                        green_line[0][col_cnt] <= green_line[1][col_cnt];
                        green_line[1][col_cnt] <= green_line[2][col_cnt];
                        green_line[2][col_cnt] <= green_data;

                        blue_line[0][col_cnt] <= blue_line[1][col_cnt];
                        blue_line[1][col_cnt] <= blue_line[2][col_cnt];
                        blue_line[2][col_cnt] <= blue_data;
                    end

                endcase

                    if (addr_cnt >= 2*IMG_WIDTH + 2 && col_cnt >= 1 && col_cnt < IMG_WIDTH-1) begin
                        window_red[0][0] <= red_line[0][col_cnt-1];
                        window_red[0][1] <= red_line[0][col_cnt];
                        window_red[0][2] <= red_line[0][col_cnt+1];
                        window_red[1][0] <= red_line[1][col_cnt-1];
                        window_red[1][1] <= red_line[1][col_cnt];
                        window_red[1][2] <= red_line[1][col_cnt+1];
                        window_red[2][0] <= red_line[2][col_cnt-1];
                        window_red[2][1] <= red_line[2][col_cnt];
                        window_red[2][2] <= red_line[2][col_cnt+1];
                        //
                        window_green[0][0] <= green_line[0][col_cnt-1];
                        window_green[0][1] <= green_line[0][col_cnt];
                        window_green[0][2] <= green_line[0][col_cnt+1];
                        window_green[1][0] <= green_line[1][col_cnt-1];
                        window_green[1][1] <= green_line[1][col_cnt];
                        window_green[1][2] <= green_line[1][col_cnt+1];
                        window_green[2][0] <= green_line[2][col_cnt-1];
                        window_green[2][1] <= green_line[2][col_cnt];
                        window_green[2][2] <= green_line[2][col_cnt+1];
                        //
                        window_blue[0][0] <= blue_line[0][col_cnt-1];
                        window_blue[0][1] <= blue_line[0][col_cnt];
                        window_blue[0][2] <= blue_line[0][col_cnt+1];
                        window_blue[1][0] <= blue_line[1][col_cnt-1];
                        window_blue[1][1] <= blue_line[1][col_cnt];
                        window_blue[1][2] <= blue_line[1][col_cnt+1];
                        window_blue[2][0] <= blue_line[2][col_cnt-1];
                        window_blue[2][1] <= blue_line[2][col_cnt];
                        window_blue[2][2] <= blue_line[2][col_cnt+1];
                    end else begin
                        window_red[0][0] <= red_data;
                        window_red[0][1] <= red_data;
                        window_red[0][2] <= red_data;
                        window_red[1][0] <= red_data;
                        window_red[1][1] <= red_data;
                        window_red[1][2] <= red_data;
                        window_red[2][0] <= red_data;
                        window_red[2][1] <= red_data;
                        window_red[2][2] <= red_data;

                        //
                        window_green[0][0] <= green_data;
                        window_green[0][1] <= green_data;
                        window_green[0][2] <= green_data;
                        window_green[1][0] <= green_data;
                        window_green[1][1] <= green_data;
                        window_green[1][2] <= green_data;
                        window_green[2][0] <= green_data;
                        window_green[2][1] <= green_data;
                        window_green[2][2] <= green_data;

                        //
                        window_blue[0][0] <= blue_data;
                        window_blue[0][1] <= blue_data;
                        window_blue[0][2] <= blue_data;
                        window_blue[1][0] <= blue_data;
                        window_blue[1][1] <= blue_data;
                        window_blue[1][2] <= blue_data;
                        window_blue[2][0] <= blue_data;
                        window_blue[2][1] <= blue_data;
                        window_blue[2][2] <= blue_data;
                    end
                end
                 ST_GAUSSIAN: begin
                tmp_gaussian_red <= 16*window_red[0][0] + 32*window_red[0][1] + 16*window_red[0][2]
                          + 32*window_red[1][0] + 64*window_red[1][1] + 32*window_red[1][2]
                          + 16*window_red[2][0] + 32*window_red[2][1] + 16*window_red[2][2];

                tmp_gaussian_green <= 16*window_green[0][0] + 32*window_green[0][1] + 16*window_green[0][2]
                          + 32*window_green[1][0] + 64*window_green[1][1] + 32*window_green[1][2]
                          + 16*window_green[2][0] + 32*window_green[2][1] + 16*window_green[2][2];

                tmp_gaussian_blue <= 16*window_blue[0][0] + 32*window_blue[0][1] + 16*window_blue[0][2]
                          + 32*window_blue[1][0] + 64*window_blue[1][1] + 32*window_blue[1][2]
                          + 16*window_blue[2][0] + 32*window_blue[2][1] + 16*window_blue[2][2];
                sel_buf <= sel_buf + 1;
                end


                
                ST_SHARPEN: begin
                    logic signed [10:0] tmp_red,tmp_green, tmp_blue;
                    tmp_red <= -window_red[0][0] - window_red[0][1] - window_red[0][2]
                          -window_red[1][0] + window_red[1][1]*9 - window_red[1][2]
                          -window_red[2][0] - window_red[2][1] - window_red[2][2];

                    tmp_green <= -window_green[0][0] - window_green[0][1] - window_green[0][2]
                          -window_green[1][0] + window_green[1][1]*9 - window_green[1][2]
                          -window_green[2][0] - window_green[2][1] - window_green[2][2];

                    tmp_blue <= -window_blue[0][0] - window_blue[0][1] - window_blue[0][2]
                          -window_blue[1][0] + window_blue[1][1]*9 - window_blue[1][2]
                          -window_blue[2][0] - window_blue[2][1] - window_blue[2][2];

                    if (tmp_red < 0) sharp_data_red <= 0;
                    else if (tmp_red > 255) sharp_data_red <= 255;
                    else sharp_data_red <= tmp_red[7:0];

                    if (tmp_green < 0) sharp_data_green <= 0;
                    else if (tmp_green > 255) sharp_data_green <= 255;
                    else sharp_data_green <= tmp_green[7:0];
                      
                    if (tmp_blue < 0) sharp_data_blue <= 0;
                   else if (tmp_blue > 255) sharp_data_blue <= 255;
                    else sharp_data_blue <= tmp_blue[7:0];

                    sel_buf <= sel_buf + 1;

                end
                ST_SEND_R: begin
                    uart_data <= sw_filter ? sharp_data_red : r8;
                    if (uart_ready && !uart_busy) begin
                        uart_start <= 1'b1;
                    end
                end

                ST_WAIT_R: ; // wait for uart_busy

                ST_SEND_G: begin
                    uart_data <= sw_filter ? sharp_data_green : g8;
                    if (uart_ready && !uart_busy) begin
                        uart_start <= 1'b1;
                    end
                end

                ST_WAIT_G: ;

                ST_SEND_B: begin
                    uart_data <= sw_filter ? sharp_data_blue : b8;
                    if (uart_ready && !uart_busy) begin
                        uart_start <= 1'b1;
                    end
                end

                ST_WAIT_B: ;

                ST_INCR: begin
                    if (addr_cnt < ROI_PIXELS - 1)
                        addr_cnt <= addr_cnt + 1;
                    else
                        send_done <= 1'b1;
                end

                ST_DONE: begin
                    // one-cycle done pulse
                end

                default: ;
            endcase
        end
    end

    // Next-state logic
    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE:       if (capture_trigger) next_state = ST_READ;
            ST_READ:       next_state = ST_WAIT;
            ST_WAIT:       next_state = ST_CAPTURE;
            ST_CAPTURE:    next_state = ST_LINEBUF;
            ST_LINEBUF:    next_state = sel_buf ? ST_SHARPEN : ST_GAUSSIAN;
            ST_GAUSSIAN:   next_state = ST_LINEBUF;
            ST_SHARPEN:    next_state = ST_SEND_R;

            ST_SEND_R:     if (uart_start)          next_state = ST_WAIT_R;
            ST_WAIT_R:     if (uart_busy)           next_state = ST_SEND_G;

            ST_SEND_G:     if (uart_start)          next_state = ST_WAIT_G;
            ST_WAIT_G:     if (uart_busy)           next_state = ST_SEND_B;

            ST_SEND_B:     if (uart_start)          next_state = ST_WAIT_B;
            ST_WAIT_B:     if (uart_busy)           next_state = ST_INCR;

            ST_INCR:       if (addr_cnt == ROI_PIXELS - 1) next_state = ST_DONE;
                           else                           next_state = ST_READ;
            ST_DONE:       next_state = ST_IDLE;
        endcase
    end

endmodule
