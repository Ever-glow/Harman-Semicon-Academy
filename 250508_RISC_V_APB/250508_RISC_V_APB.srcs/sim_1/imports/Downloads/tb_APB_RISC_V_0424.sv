`timescale 1ns / 1ps

class transaction;

    // APB Interface Signals
    rand logic [ 3:0] PADDR;
    rand logic [31:0] PWDATA;
    rand logic        PWRITE;
    rand logic        PENABLE;
    rand logic        PSEL;
    logic      [31:0] PRDATA;  // dut out data
    logic             PREADY;  // dut out data
    // outport signals
    logic      [ 3:0] fndCom;  // dut out data
    logic      [ 7:0] fndFont;  // dut out data

    constraint c_paddr_range {
        PADDR inside {4'h0, 4'h4, 4'h8};
    }
    constraint c_paddr_dist {
        PADDR dist {4'h0 := 10, 4'h4 := 50, 4'h8 := 40};
        soft PWDATA[0] dist { 1'b1 := 5, 1'b0 := 5};
    }
    constraint c_pwd_soft {
        soft PWDATA < 10;
    }

        

    task display(string name);
        $display(
            "[%s] PADDR=%h, PWDATA=%h, PWRITE=%h, PENABLE=%h, PSEL=%h, PRDATA=%h, PREADY=%h, fndCom=%h, fndFont=%h",
            name, PADDR, PWDATA, PWRITE, PENABLE, PSEL, PRDATA, PREADY, fndCom, fndFont);
    endtask  //

endclass  //transaction

interface APB_Slave_Intferface;
    logic        PCLK;
    logic        PRESET;
    // APB Interface Signals
    logic [ 3:0] PADDR;
    logic [31:0] PWDATA;
    logic        PWRITE;
    logic        PENABLE;
    logic        PSEL;
    logic [31:0] PRDATA;  // dut out data
    logic        PREADY;  // dut out data
    // outport signals
    logic [ 3:0] fndCom;  // dut out data
    logic [ 7:0] fndFont;  // dut out data

endinterface  //APB_Slave_Intferface

class generator;
    mailbox #(transaction) Gen2Drv_mbox;
    event gen_next_event;

    function new(mailbox#(transaction) Gen2Drv_mbox, event gen_next_event);
        this.Gen2Drv_mbox   = Gen2Drv_mbox;
        this.gen_next_event = gen_next_event;
    endfunction  //new()

    task run(int repeat_counter);
        transaction fnd_tr;
        repeat (repeat_counter) begin
            fnd_tr = new();  // make instrance
            if (!fnd_tr.randomize()) $error("Randomization fail!");
            fnd_tr.display("GEN");
            Gen2Drv_mbox.put(fnd_tr);
            @(gen_next_event);  // wait a event from driver
        end
    endtask  //
endclass  //generator

class driver;
    virtual APB_Slave_Intferface fnd_intf;
    mailbox #(transaction) Gen2Drv_mbox;
    transaction fnd_tr;

    function new(virtual APB_Slave_Intferface fnd_intf,
                 mailbox#(transaction) Gen2Drv_mbox);
        this.fnd_intf = fnd_intf;
        this.Gen2Drv_mbox = Gen2Drv_mbox;
    endfunction  //new()

    task run();
        forever begin
            Gen2Drv_mbox.get(fnd_tr);
            fnd_tr.display("DRV");
            @(posedge fnd_intf.PCLK);
            fnd_intf.PADDR   <= fnd_tr.PADDR;
            fnd_intf.PWDATA  <= fnd_tr.PWDATA;
            fnd_intf.PWRITE  <= 1'b1;
            fnd_intf.PENABLE <= 1'b0;
            fnd_intf.PSEL    <= 1'b1;
            @(posedge fnd_intf.PCLK);
            fnd_intf.PADDR   <= fnd_tr.PADDR;
            fnd_intf.PWDATA  <= fnd_tr.PWDATA;
            fnd_intf.PWRITE  <= 1'b1;
            fnd_intf.PENABLE <= 1'b1;
            fnd_intf.PSEL    <= 1'b1;
            wait (fnd_intf.PREADY == 1'b1);
            // @(posedge fnd_intf.PCLK);
            // @(posedge fnd_intf.PCLK);
            // @(posedge fnd_intf.PCLK);
        end
    endtask  //

endclass  //driver

class monitor;
    mailbox #(transaction) Mon2SCB_mbox;
    virtual APB_Slave_Intferface fnd_intf;
    transaction fnd_tr;

    function new(virtual APB_Slave_Intferface fnd_intf, 
                 mailbox #(transaction) Mon2SCB_mbox);
        this.fnd_intf = fnd_intf;
        this.Mon2SCB_mbox = Mon2SCB_mbox;
    endfunction //new()

    task run();
        forever begin
            fnd_tr          = new();
            @(posedge fnd_intf.PCLK);
            @(posedge fnd_intf.PREADY);
            fnd_tr.PADDR    = fnd_intf.PADDR;     //stimulus
            fnd_tr.PWDATA   = fnd_intf.PWDATA;    //stimulus
            fnd_tr.PWRITE   = fnd_intf.PWRITE;    //stimulus
            fnd_tr.PENABLE  = fnd_intf.PENABLE;   //stimulus
            fnd_tr.PSEL     = fnd_intf.PSEL;      //stimulus
            fnd_tr.PRDATA   = fnd_intf.PRDATA;
            fnd_tr.PREADY   = fnd_intf.PREADY;
            fnd_tr.fndCom   = fnd_intf.fndCom;
            fnd_tr.fndFont  = fnd_intf.fndFont;
            fnd_tr.display("MON");
            Mon2SCB_mbox.put(fnd_tr);
            // @(posedge fnd_intf.PCLK);
            // @(posedge fnd_intf.PCLK);
        end
    endtask //run

endclass //monitor

class scoreboard;
    mailbox #(transaction) Mon2SCB_mbox;
    transaction fnd_tr;
    event gen_next_event;

    int unsigned write_cnt = 0;
    int unsigned read_cnt  = 0;
    int unsigned pass_cnt  = 0;
    int unsigned fail_cnt  = 0;

    logic en_pass;
    logic font_pass;
    logic dot_pass;

    // reference model
    logic [31:0] refFndReg[0:2];
    logic [ 3:0] refDotReg;
    logic [ 7:0] refFndFont[0:15] = '{
        8'hc0,
        8'hf9,
        8'ha4,
        8'hb0,
        8'h99,
        8'h92,
        8'h82,
        8'hf8,
        8'h80,
        8'h90,
        8'h88,
        8'h83,
        8'hc6,
        8'ha1,
        8'h86,
        8'h8e
        };

    function new(mailbox #(transaction) Mon2SCB_mbox, event gen_next_event);
        this.Mon2SCB_mbox = Mon2SCB_mbox;
        this.gen_next_event = gen_next_event;

        for(int i=0; i<3; i++)begin
            refFndReg[i] = 0;
        end
        refDotReg = 0;
    endfunction //new()

    task run();
        logic [1:0] sel;
        forever begin
            Mon2SCB_mbox.get(fnd_tr);
            fnd_tr.display("SCB");
            if(fnd_tr.PWRITE) begin  // write mode
                write_cnt++;
                $display("===== SCB: Write Task #%0d =====", write_cnt);
                $display("sim time: %0t ns", $time);
                refFndReg[fnd_tr.PADDR[3:2]] = fnd_tr.PWDATA;
                case (fnd_tr.PADDR[3:2])
                    2'd0: begin
                        // FCR (enable)
                        logic exp_en = refFndReg[0][0];
                        logic [3:0] dut_comm = fnd_tr.fndCom;
                        if (exp_en) begin
                            if (dut_comm != 4'b1111) begin
                                $display("Enable PASS: fndCom=%b", dut_comm);
                                pass_cnt++;
                                $display("===== SCB: PASS #%0d =====", pass_cnt);
                            end
                            else begin
                                $error ("Enable FAIL: enable=1인데 fndCom 모두 꺼짐 (%b)", dut_comm);
                                fail_cnt++;
                            end
                        end
                        else begin
                            if (dut_comm == 4'b1111) begin
                                $display("Disable PASS: fndCom=%b", dut_comm);
                                pass_cnt++;
                                $display("===== SCB: PASS #%0d =====", pass_cnt);
                            end
                            else begin
                                $error ("Disable FAIL: enable=0인데 일부 켜짐 (%b)", dut_comm);
                                fail_cnt++;
                            end
                        end
                    end
                    2'd1: begin
                        // FDR (font data)
                        logic [3:0] idx = refFndReg[1][3:0];
                        logic [6:0] exp_seg = refFndFont[idx][6:0];
                        if (exp_seg === fnd_tr.fndFont[6:0]) begin
                            $display("Font   PASS, idx=%0d, seg=%b", idx, exp_seg);
                            pass_cnt++;
                            $display("===== SCB: PASS #%0d =====", pass_cnt);
                        end
                        else begin
                            $error("Font   FAIL: idx=%0d, exp=%b got=%b",
                                    idx, exp_seg, fnd_tr.fndFont[6:0]);
                            fail_cnt++;
                        end
                    end
                    2'd2: begin
                        // FPR (dot pattern)
                        refDotReg = fnd_tr.PWDATA[3:0];
                        unique case (fnd_tr.fndCom)
                            4'b1110: sel = 2'd0;
                            4'b1101: sel = 2'd1;
                            4'b1011: sel = 2'd2;
                            4'b0111: sel = 2'd3;
                            default: sel = 2'bxx;  // 에러 상황
                        endcase
                        if (fnd_tr.fndCom !== 4'hf) begin
                            if (fnd_tr.fndFont[7] === refDotReg[sel]) begin
                                $display("DotBit PASS: sel=%0d, exp=%b got=%b",
                                    sel, refDotReg[sel], fnd_tr.fndFont[7]);
                                pass_cnt++;
                                $display("===== SCB: PASS #%0d =====", pass_cnt);
                            end
                            else begin
                                $error("DotBit FAIL: sel=%0d, exp=%b got=%b",
                                sel, refDotReg[sel], fnd_tr.fndFont[7]);
                                fail_cnt++;
                            end
                        end
                        else begin
                            pass_cnt++;
                            $display("===== SCB: PASS #%0d =====", pass_cnt);
                        end
                    end
                endcase
            end
            
            else begin  // read mode
                
            end
            -> gen_next_event;
        end
    endtask //run

    task report();
        $display("==================================");
        $display("========== Final Report ==========");
        $display("==================================");
        $display("Write Test : %0d", write_cnt);
        $display("Read  Test : %0d", read_cnt);
        $display("PASS  Test : %0d", pass_cnt);
        $display("FAIL  Test : %0d", fail_cnt);
        $display("Total Tests: %0d", pass_cnt + fail_cnt);
        $display("==================================");
        $display("==   Test bench is finished!    ==");
        $display("==================================");
   endtask

endclass //scoreboard

class envirnment;
    mailbox #(transaction) Gen2Drv_mbox; 
    mailbox #(transaction) Mon2SCB_mbox;

    generator fnd_gen;
    driver fnd_drv;
    monitor fnd_mon;
    scoreboard fnd_scb;

    event gen_next_event;

    function new(virtual APB_Slave_Intferface fnd_intf);
        this.Gen2Drv_mbox = new();
        this.Mon2SCB_mbox = new();
        this.fnd_gen = new(Gen2Drv_mbox, gen_next_event);
        this.fnd_drv = new(fnd_intf, Gen2Drv_mbox);
        this.fnd_mon = new(fnd_intf, Mon2SCB_mbox);
        this.fnd_scb = new(Mon2SCB_mbox, gen_next_event);
    endfunction  //new()

    task run(int count);
        fork
            fnd_gen.run(count);
            fnd_drv.run();
            fnd_mon.run();
            fnd_scb.run();
        join_any
        ;
    endtask  //
endclass  //envirnment

module tb_fndController_APB_Periph ();

    envirnment fnd_env;
    APB_Slave_Intferface fnd_intf ();

    always #5 fnd_intf.PCLK = ~fnd_intf.PCLK;

    FND_Periph dut (
        // global signal
        .PCLK(fnd_intf.PCLK),
        .PRESET(fnd_intf.PRESET),
        // APB Interface Signals
        .PADDR(fnd_intf.PADDR),
        .PWDATA(fnd_intf.PWDATA),
        .PWRITE(fnd_intf.PWRITE),
        .PENABLE(fnd_intf.PENABLE),
        .PSEL(fnd_intf.PSEL),
        .PRDATA(fnd_intf.PRDATA),
        .PREADY(fnd_intf.PREADY),
        // outport signals
        .fndComm(fnd_intf.fndCom),
        .fndFont(fnd_intf.fndFont)
    );

    initial begin
        fnd_intf.PCLK   = 0;
        fnd_intf.PRESET = 1;
        #10 fnd_intf.PRESET = 0;
        fnd_env = new(fnd_intf);
        fnd_env.run(1000);
        fnd_env.fnd_scb.report();
        #30;
        $finish;
    end
endmodule
