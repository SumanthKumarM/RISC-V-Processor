//=============================================================================
// Single-program lockstep runner for real RISC-V assembly.
//
// Counterpart to risc_tb.v (which runs the hand-generated directed/random
// suite). This testbench instead loads ONE program assembled by the RISC-V
// GNU toolchain from a .mem file (see scripts/bin2mem.py) and streams it
// into the DUT's instruction memory, exactly like a risc_tb.v batch.
//
// There is no separate "toolchain execution" oracle: the toolchain only
// assembles the program, it doesn't run it. The ground truth for
// correctness is riscv_ref.v, the same behavioural golden model that
// verified 6453 hand-generated instructions with zero divergences in
// Phase 2. Lockstep comparison here reuses that same model, one retired
// instruction at a time.
//
// Halt convention: every example program must end with a self-branch/jump
// (e.g. `1: jal x0, 1b` or `beq x0,x0,0`) - the same convention risc_tb.v
// already appends to every batch. This testbench detects that condition
// automatically (next pc == the pc the halting instruction was fetched
// from) instead of relying on a fixed instruction count, since real
// programs contain loops whose retired-instruction count isn't known
// ahead of time.
//
// Usage (normally driven by `make run_asm PROG=<name>` in sim/Makefile):
//   vvp risc_tb_asm.vvp +MEMFILE=<path/to/program.mem>
//=============================================================================
`timescale 1ns/1ps

module risc_tb_asm;

    //-------------------------------------------------------------------------
    // DUT + reference model
    //-------------------------------------------------------------------------
    wire [31:0] pc,instr,ALUOUT,Jump_target,data_mem_out,write_data;
    wire [4:0]  RS1_addr,RS2_addr,RD_addr;
    wire        Branch_target,MWR,WERF;
    wire [2:0]  WBMUX;
    wire [4:0]  ALUOP;
    wire [1:0]  IRMUX,PCMUX;

    reg         clk,rst;
    reg         imem_load_en;
    reg  [31:0] imem_load_data;

    risc_v DUT(pc,instr,ALUOUT,RS1_addr,RS2_addr,RD_addr,Jump_target,Branch_target,
        MWR,WERF,data_mem_out,write_data,WBMUX,ALUOP,IRMUX,PCMUX,clk,rst,
        imem_load_en,imem_load_data);

    riscv_ref REF();

    always #5 clk = ~clk;

    localparam S_CALL_NXT_PC = 3'b110;
    localparam IMEM_WORDS    = 256;   // must match instruction_mem DEPTH (rtl/risc.v)
    localparam MAX_INSTR     = 20000; // retired-instruction safety cap (loops etc.)

    // opcodes, purely for the mnemonic decoder below
    localparam OP      = 7'b0110011;
    localparam OPIMM   = 7'b0010011;
    localparam LOAD    = 7'b0000011;
    localparam STORE   = 7'b0100011;
    localparam BRANCH  = 7'b1100011;
    localparam JAL_OP  = 7'b1101111;
    localparam JALR_OP = 7'b1100111;
    localparam LUI_OP  = 7'b0110111;
    localparam AUIPC   = 7'b0010111;

    reg [31:0] prog [0:IMEM_WORDS-1];
    integer    prog_len;
    integer    errors, checks, instr_count;
    integer    max_report;

    reg [8*1024-1:0] mem_path;

    //=========================================================================
    // mnemonic decode, purely for readable failure/trace messages
    //=========================================================================
    function [8*8:1] mnem;
        input [31:0] ir;
        reg [6:0] op, f7;
        reg [2:0] f3;
        begin
            op = ir[6:0]; f3 = ir[14:12]; f7 = ir[31:25];
            case(op)
                LUI_OP  : mnem = "lui";
                AUIPC   : mnem = "auipc";
                JAL_OP  : mnem = "jal";
                JALR_OP : mnem = "jalr";
                BRANCH  : case(f3)
                            3'b000 : mnem = "beq";
                            3'b001 : mnem = "bne";
                            3'b100 : mnem = "blt";
                            3'b101 : mnem = "bge";
                            3'b110 : mnem = "bltu";
                            3'b111 : mnem = "bgeu";
                            default: mnem = "b?";
                          endcase
                LOAD    : case(f3)
                            3'b000 : mnem = "lb";
                            3'b001 : mnem = "lh";
                            3'b010 : mnem = "lw";
                            3'b100 : mnem = "lbu";
                            3'b101 : mnem = "lhu";
                            default: mnem = "l?";
                          endcase
                STORE   : case(f3)
                            3'b000 : mnem = "sb";
                            3'b001 : mnem = "sh";
                            3'b010 : mnem = "sw";
                            default: mnem = "s?";
                          endcase
                OPIMM   : case(f3)
                            3'b000 : mnem = "addi";
                            3'b010 : mnem = "slti";
                            3'b011 : mnem = "sltiu";
                            3'b100 : mnem = "xori";
                            3'b110 : mnem = "ori";
                            3'b111 : mnem = "andi";
                            3'b001 : mnem = "slli";
                            3'b101 : mnem = (f7==7'b0100000) ? "srai" : "srli";
                            default: mnem = "i?";
                          endcase
                OP      : if(f7 == 7'b0000001)
                            case(f3)
                                3'b000 : mnem = "mul";
                                3'b001 : mnem = "mulh";
                                3'b010 : mnem = "mulhsu";
                                3'b011 : mnem = "mulhu";
                                3'b100 : mnem = "div";
                                3'b101 : mnem = "divu";
                                3'b110 : mnem = "rem";
                                3'b111 : mnem = "remu";
                            endcase
                          else
                            case(f3)
                                3'b000 : mnem = (f7==7'b0100000) ? "sub" : "add";
                                3'b001 : mnem = "sll";
                                3'b010 : mnem = "slt";
                                3'b011 : mnem = "sltu";
                                3'b100 : mnem = "xor";
                                3'b101 : mnem = (f7==7'b0100000) ? "sra" : "srl";
                                3'b110 : mnem = "or";
                                3'b111 : mnem = "and";
                            endcase
                default : mnem = "???";
            endcase
        end
    endfunction

    //=========================================================================
    // load the .mem file named by +MEMFILE=<path> into prog[]
    //=========================================================================
    task load_program;
        integer fd, rc, done;
        reg [31:0] w;
        begin
            prog_len = 0;
            fd = $fopen(mem_path, "r");
            if (fd == 0) begin
                $display("ERROR: could not open MEMFILE '%0s'", mem_path);
                $finish;
            end
            done = 0;
            while (!done) begin
                rc = $fscanf(fd, "%h", w);
                if (rc == 1) begin
                    if (prog_len >= IMEM_WORDS) begin
                        $display("ERROR: program exceeds %0d-word instruction memory", IMEM_WORDS);
                        $fclose(fd);
                        $finish;
                    end
                    prog[prog_len] = w;
                    prog_len       = prog_len + 1;
                end
                else done = 1;
            end
            $fclose(fd);
            if (prog_len == 0) begin
                $display("ERROR: no instructions read from MEMFILE '%0s'", mem_path);
                $finish;
            end
            $display("Loaded %0d instruction(s) from %0s", prog_len, mem_path);
        end
    endtask

    //=========================================================================
    // execution control (same shape as risc_tb.v)
    //=========================================================================
    task wait_retire;
        output reg timeout;
        integer guard;
        begin
            timeout = 1'b0;
            guard   = 0;
            while((DUT.FSM.present_state !== S_CALL_NXT_PC) && (guard < 400)) begin
                @(posedge clk);
                guard = guard + 1;
            end
            if(guard >= 400) timeout = 1'b1;
            else begin
                @(posedge clk);
                #1;
            end
        end
    endtask

    task compare;
        input integer idx;
        input [31:0] ir_pc;
        input [31:0] ir;
        integer i;
        reg     bad;
        begin
            bad    = 1'b0;
            checks = checks + 1;

            if(pc !== REF.pc) bad = 1'b1;
            for(i=0;i<32;i=i+1)
                if(DUT.register_file.mem[i] !== REF.regs[i]) bad = 1'b1;
            for(i=0;i<64;i=i+1)
                if(DUT.data_memory.mem[i] !== REF.dmem[i]) bad = 1'b1;

            if(bad) begin
                errors = errors + 1;
                if(errors <= max_report) begin
                    $display("\n    ERROR #%0d @ instr %0d", errors, idx);
                    $display("      pc=%h  ir=%h (%0s)", ir_pc, ir, mnem(ir));
                    $display("      rd=x%0d rs1=x%0d rs2=x%0d", ir[11:7], ir[19:15], ir[24:20]);
                    if(pc !== REF.pc)
                        $display("      PC  : dut=%h ref=%h", pc, REF.pc);
                    for(i=0;i<32;i=i+1)
                        if(DUT.register_file.mem[i] !== REF.regs[i])
                            $display("      x%-2d : dut=%h ref=%h", i, DUT.register_file.mem[i], REF.regs[i]);
                    for(i=0;i<64;i=i+1)
                        if(DUT.data_memory.mem[i] !== REF.dmem[i])
                            $display("      mem[%0d] : dut=%h ref=%h", i*4, DUT.data_memory.mem[i], REF.dmem[i]);
                    if(errors == max_report)
                        $display("      ... suppressing further details ...");
                end
            end
        end
    endtask

    task dump_state;
        integer i;
        begin
            $display("\n---------------------------------------------------------");
            $display(" Final architectural state (DUT)");
            $display("---------------------------------------------------------");
            $display(" pc = %h", pc);
            for(i=0;i<32;i=i+4)
                $display(" x%-2d=%h  x%-2d=%h  x%-2d=%h  x%-2d=%h",
                    i,DUT.register_file.mem[i], i+1,DUT.register_file.mem[i+1],
                    i+2,DUT.register_file.mem[i+2], i+3,DUT.register_file.mem[i+3]);
            $display(" data memory (non-zero words):");
            for(i=0;i<64;i=i+1)
                if(DUT.data_memory.mem[i] !== 32'd0)
                    $display("   mem[%0d] = %h", i*4, DUT.data_memory.mem[i]);
            $display("---------------------------------------------------------");
        end
    endtask

    task run_program;
        integer i;
        reg     timeout, bail;
        reg [31:0] ir_pc, ir;
        begin
            // ---- 1. reset the DUT
            rst          = 1'b1;
            imem_load_en = 1'b0;
            @(negedge clk);
            @(negedge clk);

            // ---- 2. stream the program in, mirroring it into the model
            imem_load_en = 1'b1;
            for(i=0;i<prog_len;i=i+1) begin
                imem_load_data = prog[i];
                REF.load_word(i, prog[i]);
                @(negedge clk);
            end
            imem_load_en = 1'b0;

            // ---- 3. release the core; both sides start from all-zero state
            @(negedge clk);
            rst = 1'b0;
            REF.reset_state;
            for(i=0;i<32;i=i+1) begin
                DUT.register_file.mem[i] = 32'd0;
                REF.regs[i]              = 32'd0;
            end
            for(i=0;i<64;i=i+1) begin
                DUT.data_memory.mem[i] = 32'd0;
                REF.dmem[i]            = 32'd0;
            end

            // ---- 4. lockstep execution until the halt self-loop or MAX_INSTR
            bail = 1'b0;
            i    = 0;
            while(!bail && (i < MAX_INSTR)) begin
                ir_pc = REF.pc;
                ir    = REF.imem[REF.pc[9:2]];
                wait_retire(timeout);
                if(timeout) begin
                    errors = errors + 1;
                    $display("    [%0d] TIMEOUT at pc=%h : %0s (unsupported opcode or stall?)", i, ir_pc, mnem(ir));
                    bail = 1'b1;
                end
                else begin
                    REF.step;
                    instr_count = instr_count + 1;
                    compare(i, ir_pc, ir);
                    if((i % 50) == 49)
                        $write("[%0d]", i+1);
                    else if((i % 10) == 9)
                        $write(".");
                    $fflush();
                    if(REF.pc == ir_pc) begin
                        $display("\n  halt (self-loop) detected at pc=%h after %0d instruction(s)", ir_pc, i+1);
                        bail = 1'b1;
                    end
                end
                i = i + 1;
            end
            if(!bail)
                $display("\n  MAX_INSTR (%0d) reached without hitting the halt self-loop", MAX_INSTR);
        end
    endtask

    //=========================================================================
    // main
    //=========================================================================
    initial begin
`ifdef DUMP
        $dumpfile("risc_tb_asm.vcd");
        $dumpvars(0, risc_tb_asm);
`endif
        clk            = 1'b0;
        rst            = 1'b1;
        imem_load_en   = 1'b0;
        imem_load_data = 32'd0;
        errors         = 0;
        checks         = 0;
        instr_count    = 0;
        max_report     = 25;

        if(!$value$plusargs("MEMFILE=%s", mem_path)) begin
            $display("ERROR: usage: +MEMFILE=<path/to/program.mem>");
            $finish;
        end

        $display("=========================================================");
        $display(" RV32I + RV32M core: single-program lockstep run");
        $display(" Reference: riscv_ref.v (same golden model as risc_tb.v)");
        $display("=========================================================\n");

        load_program;
        run_program;

        $display("=========================================================");
        $display(" RUN SUMMARY");
        $display("=========================================================");
        $display(" instructions retired : %0d", instr_count);
        $display(" state checks         : %0d (pc + regs + dmem)", checks);
        $display(" errors found         : %0d", errors);
        if(errors == 0) begin
            $display(" ");
            $display(" ✓ PASS  DUT matched the golden model on every retired instruction");
        end
        else begin
            $display(" ");
            $display(" ✗ FAIL  %0d instruction(s) diverged from the golden model", errors);
        end
        dump_state;
        $display("=========================================================");
        $display("TB_STATUS: %0s", errors == 0 ? "PASS" : "FAIL");
        $finish;
    end

    // global watchdog
    initial begin
        #50_000_000;
        $display("GLOBAL TIMEOUT - simulation did not finish");
        $finish;
    end

endmodule
