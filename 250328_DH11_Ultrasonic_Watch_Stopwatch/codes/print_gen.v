`timescale 1ns / 1ps

module top_print_gen(
    input clk,
    input reset,
    input start,       
    input [8:0] distance_bin, // 측정된 거리 값 (8비트, 0~255)
    input [1:0] mode,
    input [5:0] sec,
    input [5:0] min,
    input [4:0] hour,
    input [7:0] humid,
    input [7:0] temp,
    output reg [7:0] tx_data, // FIFO TX에 전달할 데이터
    output reg tx_wr,         // FIFO TX에 쓰기 신호
    output reg done,           // 전체 문자열 전송 완료 플래그
    output reg busy
);

    // port
    wire [7:0] w_tx_data, w_tx_data_dist, w_tx_data_time, w_tx_data_th;
    wire w_tx_wr, w_tx_wr_dist, w_tx_wr_time, w_tx_wr_th;
    wire w_done, w_done_dist, w_done_time, w_done_th;
    wire w_busy, w_busy_dist, w_busy_time, w_busy_th;

    reg [1:0] enable;
    // mode 입력에 따라 HHMMSS / dist / temp&humid 문자열 생성
    always @(*) begin
        enable = 2'd0;
        case (mode)
            2'd0, 2'd1: enable = 2'd0;  // stopwatch or watch
            2'd2      : enable = 2'd1;  // dist
            2'd3      : enable = 2'd2;  // temp&humid
        endcase
    end
    
    assign {w_tx_data, w_tx_wr, w_done, w_busy} =
            (enable == 2'd0) ? {w_tx_data_time, w_tx_wr_time, w_done_time, w_busy_time} : 
            (enable == 2'd1) ? {w_tx_data_dist, w_tx_wr_dist, w_done_dist, w_busy_dist} :
            (enable == 2'd2) ? {w_tx_data_th, w_tx_wr_th, w_done_th, w_busy_th}         :
                               {w_tx_data_time, w_tx_wr_time, w_done_time, w_busy_time} ; 


    print_gen_dist U_PRINT_DIST(
        .clk(clk),
        .reset(reset),
        .start(start),       
        .distance_bin(distance_bin), 
        .tx_data(w_tx_data),
        .tx_wr(w_tx_wr),       
        .done(w_done),           
        .busy(w_busy)
    );

    print_gen_time();

    print_gen_temp_humid();


endmodule


module print_gen_dist(
    input clk,
    input reset,
    input start,       
    input [8:0] distance_bin, // 측정된 거리 값 (8비트, 0~255)
    output reg [7:0] tx_data, // FIFO TX에 전달할 데이터
    output reg tx_wr,         // FIFO TX에 쓰기 신호
    output reg done,           // 전체 문자열 전송 완료 플래그
    output reg busy
);

    // FSM 상태 정의
    localparam S_IDLE           = 3'd0,
               S_SEND_PREFIX    = 3'd1,
               S_COMPUTE_DIGITS = 3'd2,
               S_SEND_DIGITS    = 3'd3,
               S_DONE           = 3'd4;
               
    reg [2:0] state, next_state;
    
    // 접두어 문자열 "distance: " (10글자)
    reg [7:0] prefix_dist [0:9];
    initial begin
        prefix_dist[1] = "d";  // 0x64
        prefix_dist[2] = "i";  // 0x69
        prefix_dist[3] = "s";  // 0x73
        prefix_dist[4] = "t";  // 0x74
        prefix_dist[5] = "a";  // 0x61
        prefix_dist[6] = "n";  // 0x6E
        prefix_dist[7] = "c";  // 0x63
        prefix_dist[8] = "e";  // 0x65
        prefix_dist[9] = ":";  // 0x3A
        prefix_dist[0] = " ";
    end

    // 인덱스 변수: 접두어 전송 및 숫자 전송을 위한 인덱스
    reg [3:0] prefix_idx, next_prefix_idx; // 0~8
    reg [1:0] digit_idx, next_digit_idx;     // 0: 백, 1: 십, 2: 일

    // 숫자 계산을 위한 BCD 값
    reg [3:0] hundreds, tens, ones;
    reg [3:0] next_hundreds, next_tens, next_ones;
    
    // 내부에 저장된 거리값; start 신호 발생 시 캡처
    reg [8:0] stored_distance, next_stored_distance;
    
    // 출력 신호의 next 값
    reg [7:0] next_tx_data;
    reg next_tx_wr, next_done;
    reg next_busy;
    
    // 순차 로직: 상태, 인덱스, 내부 레지스터 업데이트
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state           <= S_IDLE;
            prefix_idx      <= 0;
            digit_idx       <= 0;
            tx_data         <= 8'd0;
            tx_wr           <= 1'b0;
            done            <= 1'b0;
            hundreds      <= 4'd0;
            tens          <= 4'd0;
            ones          <= 4'd0;
            stored_distance <= 9'd0;
            busy            <= 1'b0;
        end else begin
            state           <= next_state;
            prefix_idx      <= next_prefix_idx;
            digit_idx       <= next_digit_idx;
            tx_data         <= next_tx_data;
            tx_wr           <= next_tx_wr;
            done            <= next_done;
            hundreds        <= next_hundreds;
            tens            <= next_tens;
            ones            <= next_ones;
            stored_distance <= next_stored_distance;
            busy            <= next_busy;
        end
    end

    // 조합 로직: 다음 상태 및 출력 신호 결정
    always @(*) begin
        // 기본값 할당: 이전 값 유지
        next_state       = state;
        next_prefix_idx  = prefix_idx;
        next_digit_idx   = digit_idx;
        next_tx_data     = tx_data;
        next_tx_wr       = tx_wr;
        next_done        = done;
        next_hundreds    = hundreds;
        next_tens        = tens;
        next_ones        = ones;
        next_stored_distance = stored_distance;
        next_busy = busy;
        
        case (state)
            S_IDLE: begin
                // FSM 대기: start 신호가 원샷으로 들어오면 새로운 명령으로 인식
                next_prefix_idx = 0;
                next_digit_idx  = 0;
                next_done = 1'b0;
                next_busy = 1'b0;
                if (start) begin
                    // 새 명령 수신 시, 입력된 거리 값을 캡처하고 인덱스 초기화
                    next_stored_distance = distance_bin;
                    next_state = S_SEND_PREFIX;
                end
            end
            
            S_SEND_PREFIX: begin
                // 접두어 문자 전송 ("distance:")
                next_tx_data = prefix_dist[prefix_idx];
                next_busy = 1'b1;
                if (prefix_idx == 9) begin
                    next_state = S_COMPUTE_DIGITS;
                end
                else
                    next_prefix_idx = prefix_idx + 1;
            end
            
            S_COMPUTE_DIGITS: begin
                // 저장된 거리값을 기준으로 백, 십, 일 자리 계산
                next_busy = 1'b1;
                next_tx_data = prefix_dist[0];
                next_hundreds = stored_distance / 100;
                next_tens     = (stored_distance % 100) / 10;
                next_ones     = stored_distance % 10;
                next_digit_idx = 0;
                next_state = S_SEND_DIGITS;
            end
            
            S_SEND_DIGITS: begin
                next_busy = 1'b1;
                // 숫자 전송: 각 자리값을 ASCII('0' + digit)로 변환하여 전송
                case (digit_idx)
                    2'd0: next_tx_data = hundreds + 8'h30;
                    2'd1: next_tx_data = tens + 8'h30;
                    2'd2: next_tx_data = ones + 8'h30;
                    default: next_tx_data = 8'h00;
                endcase
                if (digit_idx == 2) begin
                    next_state = S_DONE;
                    next_digit_idx = 0;
                end
                else
                    next_digit_idx = digit_idx + 1;
            end
            
            S_DONE: begin
                next_busy = 1'b0;
                next_done = 1'b1;
                next_state = S_IDLE;
            end
        endcase
    end

endmodule

module print_gen_time(
    input clk,
    input reset,
    input start,       
    input [7:0] humid,
    input [7:0] temp, 
    output reg [7:0] tx_data, // FIFO TX에 전달할 데이터
    output reg tx_wr,         // FIFO TX에 쓰기 신호
    output reg done,           // 전체 문자열 전송 완료 플래그
    output reg busy
);

    // FSM 상태 정의
    localparam S_IDLE           = 3'd0,
               S_SEND_PREFIX    = 3'd1,
               S_COMPUTE_DIGITS = 3'd2,
               S_SEND_DIGITS    = 3'd3,
               S_DONE           = 3'd4;
               
    reg [2:0] state, next_state;
    
    // 접두어 문자열 "temp: " (6글자)
    reg [7:0] prefix_temp [0:5];
    initial begin
        prefix_temp[1] = "t";  
        prefix_temp[2] = "e";  
        prefix_temp[3] = "m";  
        prefix_temp[4] = "p";  
        prefix_temp[5] = ":";  
        prefix_temp[0] = " ";
    end

    // 접두어 문자열 "temp: " (7글자)
    reg [7:0] prefix_humid [0:6];
    initial begin
        prefix_humid[1] = "h"; 
        prefix_humid[2] = "u"; 
        prefix_humid[3] = "m"; 
        prefix_humid[4] = "i"; 
        prefix_humid[5] = "d"; 
        prefix_humid[6] = ":";
        prefix_humid[0] = " ";
    end

    // 인덱스 변수: 접두어 전송 및 숫자 전송을 위한 인덱스
    reg [3:0] prefix_idx, next_prefix_idx; //
    reg [1:0] digit_idx, next_digit_idx;     // 0: 백, 1: 십

    // 숫자 계산을 위한 BCD 값
    reg [3:0] tens, ones;
    reg [3:0] next_tens, next_ones;
    
    // 내부에 저장된 거리값; start 신호 발생 시 캡처
    reg [7:0] temp, next_temp;
    reg [7:0] humid, next_humid;
    
    // 출력 신호의 next 값
    reg [7:0] next_tx_data;
    reg next_tx_wr, next_done;
    reg next_busy;
    
    // 순차 로직: 상태, 인덱스, 내부 레지스터 업데이트
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state           <= S_IDLE;
            prefix_idx      <= 0;
            digit_idx       <= 0;
            tx_data         <= 8'd0;
            tx_wr           <= 1'b0;
            done            <= 1'b0;
            tens          <= 4'd0;
            ones          <= 4'd0;
            temp          <= 8'd0;
            humid         <= 8'd0;
            busy            <= 1'b0;
        end else begin
            state           <= next_state;
            prefix_idx      <= next_prefix_idx;
            digit_idx       <= next_digit_idx;
            tx_data         <= next_tx_data;
            tx_wr           <= next_tx_wr;
            done            <= next_done;
            temp            <= next_temp;
            humid           <= next_humid;
            tens            <= next_tens;
            ones            <= next_ones;
            busy            <= next_busy;
        end
    end

    // 조합 로직: 다음 상태 및 출력 신호 결정
    always @(*) begin
        // 기본값 할당: 이전 값 유지
        next_state       = state;
        next_prefix_idx  = prefix_idx;
        next_digit_idx   = digit_idx;
        next_tx_data     = tx_data;
        next_tx_wr       = tx_wr;
        next_done        = done;
        next_tens        = tens;
        next_ones        = ones;
        next_temp        = temp;
        next_humid       = humid;
        next_busy        = busy;
        
        case (state)
            S_IDLE: begin
                // FSM 대기: start 신호가 원샷으로 들어오면 새로운 명령으로 인식
                next_prefix_idx = 0;
                next_digit_idx  = 0;
                next_done = 1'b0;
                next_busy = 1'b0;
                if (start) begin
                    // 새 명령 수신 시, 입력된 거리 값을 캡처하고 인덱스 초기화
                    next_humid = humid;
                    next_state = S_SEND_PREFIX;
                end
            end
            
            S_SEND_PREFIX: begin
                // 접두어 문자 전송 ("distance:")
                next_tx_data = prefix[prefix_idx];
                next_busy = 1'b1;
                if (prefix_idx == 9) begin
                    next_state = S_COMPUTE_DIGITS;
                end
                else
                    next_prefix_idx = prefix_idx + 1;
            end
            
            S_COMPUTE_DIGITS: begin
                // 저장된 거리값을 기준으로 백, 십, 일 자리 계산
                next_busy = 1'b1;
                next_tx_data = prefix[0];
                next_hundreds = stored_distance / 100;
                next_tens     = (stored_distance % 100) / 10;
                next_ones     = stored_distance % 10;
                next_digit_idx = 0;
                next_state = S_SEND_DIGITS;
            end
            
            S_SEND_DIGITS: begin
                next_busy = 1'b1;
                // 숫자 전송: 각 자리값을 ASCII('0' + digit)로 변환하여 전송
                case (digit_idx)
                    2'd0: next_tx_data = hundreds + 8'h30;
                    2'd1: next_tx_data = tens + 8'h30;
                    2'd2: next_tx_data = ones + 8'h30;
                    default: next_tx_data = 8'h00;
                endcase
                if (digit_idx == 2) begin
                    next_state = S_DONE;
                    next_digit_idx = 0;
                end
                else
                    next_digit_idx = digit_idx + 1;
            end
            
            S_DONE: begin
                next_busy = 1'b0;
                next_done = 1'b1;
                next_state = S_IDLE;
            end
        endcase
    end

endmodule

module print_gen_dist(
    input clk,
    input reset,
    input start,       
    input [8:0] distance_bin, // 측정된 거리 값 (8비트, 0~255)
    output reg [7:0] tx_data, // FIFO TX에 전달할 데이터
    output reg tx_wr,         // FIFO TX에 쓰기 신호
    output reg done,           // 전체 문자열 전송 완료 플래그
    output reg busy
);

    // FSM 상태 정의
    localparam S_IDLE           = 3'd0,
               S_SEND_PREFIX    = 3'd1,
               S_COMPUTE_DIGITS = 3'd2,
               S_SEND_DIGITS    = 3'd3,
               S_DONE           = 3'd4;
               
    reg [2:0] state, next_state;
    
    // 접두어 문자열 "distance: " (10글자)
    reg [7:0] prefix_dist [0:9];
    initial begin
        prefix_dist[1] = "d";  // 0x64
        prefix_dist[2] = "i";  // 0x69
        prefix_dist[3] = "s";  // 0x73
        prefix_dist[4] = "t";  // 0x74
        prefix_dist[5] = "a";  // 0x61
        prefix_dist[6] = "n";  // 0x6E
        prefix_dist[7] = "c";  // 0x63
        prefix_dist[8] = "e";  // 0x65
        prefix_dist[9] = ":";  // 0x3A
        prefix_dist[0] = " ";
    end

    // 접두어 문자열 "humid: " (7글자)
    reg [7:0] prefix_humid [0:6];
    initial begin
        prefix_humid[1] = "h"; 
        prefix_humid[2] = "u"; 
        prefix_humid[3] = "m"; 
        prefix_humid[4] = "i"; 
        prefix_humid[5] = "d"; 
        prefix_humid[6] = ":";
        prefix_humid[0] = " ";
    end


    // 인덱스 변수: 접두어 전송 및 숫자 전송을 위한 인덱스
    reg [3:0] prefix_idx, next_prefix_idx; // 0~8
    reg [1:0] digit_idx, next_digit_idx;     // 0: 백, 1: 십, 2: 일

    // 숫자 계산을 위한 BCD 값
    reg [3:0] hundreds, tens, ones;
    reg [3:0] next_hundreds, next_tens, next_ones;
    
    // 내부에 저장된 거리값; start 신호 발생 시 캡처
    reg [8:0] stored_distance, next_stored_distance;
    
    // 출력 신호의 next 값
    reg [7:0] next_tx_data;
    reg next_tx_wr, next_done;
    reg next_busy;
    
    // 순차 로직: 상태, 인덱스, 내부 레지스터 업데이트
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state           <= S_IDLE;
            prefix_idx      <= 0;
            digit_idx       <= 0;
            tx_data         <= 8'd0;
            tx_wr           <= 1'b0;
            done            <= 1'b0;
            hundreds      <= 4'd0;
            tens          <= 4'd0;
            ones          <= 4'd0;
            stored_distance <= 9'd0;
            busy            <= 1'b0;
        end else begin
            state           <= next_state;
            prefix_idx      <= next_prefix_idx;
            digit_idx       <= next_digit_idx;
            tx_data         <= next_tx_data;
            tx_wr           <= next_tx_wr;
            done            <= next_done;
            hundreds        <= next_hundreds;
            tens            <= next_tens;
            ones            <= next_ones;
            stored_distance <= next_stored_distance;
            busy            <= next_busy;
        end
    end

    // 조합 로직: 다음 상태 및 출력 신호 결정
    always @(*) begin
        // 기본값 할당: 이전 값 유지
        next_state       = state;
        next_prefix_idx  = prefix_idx;
        next_digit_idx   = digit_idx;
        next_tx_data     = tx_data;
        next_tx_wr       = tx_wr;
        next_done        = done;
        next_hundreds    = hundreds;
        next_tens        = tens;
        next_ones        = ones;
        next_stored_distance = stored_distance;
        next_busy = busy;
        
        case (state)
            S_IDLE: begin
                // FSM 대기: start 신호가 원샷으로 들어오면 새로운 명령으로 인식
                next_prefix_idx = 0;
                next_digit_idx  = 0;
                next_done = 1'b0;
                next_busy = 1'b0;
                if (start) begin
                    // 새 명령 수신 시, 입력된 거리 값을 캡처하고 인덱스 초기화
                    next_stored_distance = distance_bin;
                    next_state = S_SEND_PREFIX;
                end
            end
            
            S_SEND_PREFIX: begin
                // 접두어 문자 전송 ("distance:")
                next_tx_data = prefix[prefix_idx];
                next_busy = 1'b1;
                if (prefix_idx == 9) begin
                    next_state = S_COMPUTE_DIGITS;
                end
                else
                    next_prefix_idx = prefix_idx + 1;
            end
            
            S_COMPUTE_DIGITS: begin
                // 저장된 거리값을 기준으로 백, 십, 일 자리 계산
                next_busy = 1'b1;
                next_tx_data = prefix[0];
                next_hundreds = stored_distance / 100;
                next_tens     = (stored_distance % 100) / 10;
                next_ones     = stored_distance % 10;
                next_digit_idx = 0;
                next_state = S_SEND_DIGITS;
            end
            
            S_SEND_DIGITS: begin
                next_busy = 1'b1;
                // 숫자 전송: 각 자리값을 ASCII('0' + digit)로 변환하여 전송
                case (digit_idx)
                    2'd0: next_tx_data = hundreds + 8'h30;
                    2'd1: next_tx_data = tens + 8'h30;
                    2'd2: next_tx_data = ones + 8'h30;
                    default: next_tx_data = 8'h00;
                endcase
                if (digit_idx == 2) begin
                    next_state = S_DONE;
                    next_digit_idx = 0;
                end
                else
                    next_digit_idx = digit_idx + 1;
            end
            
            S_DONE: begin
                next_busy = 1'b0;
                next_done = 1'b1;
                next_state = S_IDLE;
            end
        endcase
    end

endmodule