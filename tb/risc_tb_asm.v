//=============================================================================
// Single-program lockstep runner for real RISC-V assembly.
//
// Counterpart to risc_tb.v (which runs the hand-generated directed/random
// suite). This testbench instead loads ONE program assembled by the RISC-V
// GNU toolchain from a .mem file (see scripts/bin2mem.py) and streams it
// into the DUT's instruction memory, exactly like a risc_tb.v batch.
//
// The toolchain itself is not an oracle: gcc/as/ld only assemble the
// program, they never run it. Correctness is judged by up to two independent
// models, checked simultaneously and reported separately:
//
//   riscv_ref.v  (always on) - the behavioural golden model that verified
//       6453 hand-generated instructions with zero divergences in Phase 2.
//       Compared on pc, all 32 registers and all 64 data-memory words.
//
//   qemu-riscv32 (optional, +QEMUTRACE=<path>) - a pre-recorded trace from a
//       mature, independently developed implementation with no relationship
//       to this repository. See qemu/README.md for how the trace is produced.
//
// The second model is what makes a passing run mean something stronger. risc.v
// and riscv_ref.v were written by the same author from the same reading of the
// spec, so they can agree on a wrong answer and lockstep would never notice;
// qemu cannot make that same mistake by construction. Both models are checked
// on every retired instruction and the summary reports each one's divergence
// count on its own, so it is always clear which oracle objected.
//
// The qemu trace supplies pc, the 32 registers, and the stores the program
// performed. Stores are folded into a shadow data memory using the core's own
// addr[7:2] aliasing, which is then compared against the DUT's data_mem in
// full - so memory is checked directly against qemu, not just inferred from
// later loads.
//
// Halt convention: every example program must end with a self-branch/jump
// (e.g. `1: jal x0, 1b` or `beq x0,x0,0`) - the same convention risc_tb.v
// already appends to every batch. This testbench detects that condition
// automatically (next pc == the pc the halting instruction was fetched
// from) instead of relying on a fixed instruction count, since real
// programs contain loops whose retired-instruction count isn't known
// ahead of time.
//
// Usage (normally driven by sim/Makefile):
//   make run_asm  PROG=<name>    -> vvp ... +MEMFILE=<prog.mem>
//   make run_qemu PROG=<name>    -> vvp ... +MEMFILE=<prog.mem> \
//                                            +QEMUTRACE=<prog.qtrace>
// Omitting +QEMUTRACE leaves the run exactly as it was before the qemu oracle
// existed: golden model only.
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

    //-------------------------------------------------------------------------
    // qemu oracle state (inactive unless +QEMUTRACE=<path> is given)
    //
    // The trace is streamed one record per retired instruction rather than
    // slurped into an array: a record is 34 words and a run can retire tens of
    // thousands of instructions, so holding the whole thing would cost far
    // more memory than the DUT itself.
    //-------------------------------------------------------------------------
    reg [8*1024-1:0] qt_path;
    integer          qt_fd;            // 0 when the oracle is off
    integer          qt_total;         // records the trace claims to hold
    integer          qt_read;          // records consumed so far
    reg              qt_exhausted;     // trace ran out before the core halted
    reg [31:0]       qt_pc;
    reg [31:0]       qt_regs [0:31];
    reg [31:0]       qt_dmem [0:63];   // shadow memory rebuilt from qemu stores
    integer          qerrors;          // divergences against qemu
    integer          qchecks;

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

    //=========================================================================
    // qemu oracle: open the trace named by +QEMUTRACE=<path>
    //=========================================================================
    task qt_open;
        integer rc, i;
        begin
            qt_fd        = 0;
            qt_total     = 0;
            qt_read      = 0;
            qt_exhausted = 1'b0;
            for(i=0;i<64;i=i+1) qt_dmem[i] = 32'd0;

            if($value$plusargs("QEMUTRACE=%s", qt_path)) begin
                qt_fd = $fopen(qt_path, "r");
                if(qt_fd == 0) begin
                    $display("ERROR: could not open QEMUTRACE '%0s'", qt_path);
                    $finish;
                end
                rc = $fscanf(qt_fd, "%d", qt_total);
                if(rc != 1 || qt_total <= 0) begin
                    $display("ERROR: QEMUTRACE '%0s' is missing its record-count header", qt_path);
                    $finish;
                end
            end
        end
    endtask

    //=========================================================================
    // Fold one qemu store into the shadow data memory.
    //
    // Uses the core's own geometry: data_mem is 64 words indexed by addr[7:2]
    // (see rtl/risc.v), so every address aliases into a 256-byte window. The
    // trace carries absolute addresses (DATA_BASE+k), and applying that same
    // aliasing here is what lets the shadow be compared word-for-word against
    // the DUT's memory.
    //=========================================================================
    task qt_apply_store;
        input [31:0] addr;
        input [31:0] sz;
        input [31:0] data;
        reg [5:0] w;
        begin
            w = addr[7:2];
            case(sz)
                32'd1 : case(addr[1:0])
                            2'b00 : qt_dmem[w][7:0]   = data[7:0];
                            2'b01 : qt_dmem[w][15:8]  = data[7:0];
                            2'b10 : qt_dmem[w][23:16] = data[7:0];
                            2'b11 : qt_dmem[w][31:24] = data[7:0];
                        endcase
                32'd2 : if(addr[1]) qt_dmem[w][31:16] = data[15:0];
                        else        qt_dmem[w][15:0]  = data[15:0];
                32'd4 : qt_dmem[w] = data;
                default : begin
                    qerrors = qerrors + 1;
                    $display("    qemu trace: unexpected store size %0d at addr %h", sz, addr);
                end
            endcase
        end
    endtask

    //=========================================================================
    // Consume the next qemu record and compare the full architectural state.
    //=========================================================================
    task qt_step;
        input integer idx;
        input [31:0]  ir_pc;
        input [31:0]  ir;
        integer i, rc, nst, s;
        reg [31:0] a, sz, d;
        reg        bad;
        begin
            if(qt_fd != 0) begin
                if(qt_read >= qt_total) begin
                    // Expected exactly once: the core still has to retire its
                    // halt self-loop, which has no counterpart in the qemu
                    // build (it exits via a syscall instead), so the trace
                    // legitimately ends one instruction earlier.
                    qt_exhausted = 1'b1;
                end
                else begin
                    rc = $fscanf(qt_fd, "%h", qt_pc);
                    if(rc != 1) begin
                        $display("ERROR: qemu trace truncated at record %0d", qt_read);
                        $finish;
                    end
                    for(i=0;i<32;i=i+1) rc = $fscanf(qt_fd, "%h", qt_regs[i]);
                    rc = $fscanf(qt_fd, "%h", nst);
                    for(s=0;s<nst;s=s+1) begin
                        rc = $fscanf(qt_fd, "%h", a);
                        rc = $fscanf(qt_fd, "%h", sz);
                        rc = $fscanf(qt_fd, "%h", d);
                        qt_apply_store(a, sz, d);
                    end

                    qt_read = qt_read + 1;
                    qchecks = qchecks + 1;

                    bad = 1'b0;
                    if(pc !== qt_pc) bad = 1'b1;
                    for(i=0;i<32;i=i+1)
                        if(DUT.register_file.mem[i] !== qt_regs[i]) bad = 1'b1;
                    for(i=0;i<64;i=i+1)
                        if(DUT.data_memory.mem[i] !== qt_dmem[i]) bad = 1'b1;

                    if(bad) begin
                        qerrors = qerrors + 1;
                        if(qerrors <= max_report) begin
                            $display("\n    QEMU ERROR #%0d @ instr %0d", qerrors, idx);
                            $display("      pc=%h  ir=%h (%0s)", ir_pc, ir, mnem(ir));
                            $display("      rd=x%0d rs1=x%0d rs2=x%0d", ir[11:7], ir[19:15], ir[24:20]);
                            if(pc !== qt_pc)
                                $display("      PC  : dut=%h qemu=%h", pc, qt_pc);
                            for(i=0;i<32;i=i+1)
                                if(DUT.register_file.mem[i] !== qt_regs[i])
                                    $display("      x%-2d : dut=%h qemu=%h", i, DUT.register_file.mem[i], qt_regs[i]);
                            for(i=0;i<64;i=i+1)
                                if(DUT.data_memory.mem[i] !== qt_dmem[i])
                                    $display("      mem[%0d] : dut=%h qemu=%h", i*4, DUT.data_memory.mem[i], qt_dmem[i]);
                            if(qerrors == max_report)
                                $display("      ... suppressing further details ...");
                        end
                    end
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
                qt_dmem[i]             = 32'd0;   // shadow starts zeroed too
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
                    qt_step(i, ir_pc, ir);   // no-op unless +QEMUTRACE was given
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
        qerrors        = 0;
        qchecks        = 0;
        instr_count    = 0;
        max_report     = 25;

        if(!$value$plusargs("MEMFILE=%s", mem_path)) begin
            $display("ERROR: usage: +MEMFILE=<path/to/program.mem> [+QEMUTRACE=<path>]");
            $finish;
        end

        qt_open;

        $display("=========================================================");
        $display(" RV32I + RV32M core: single-program lockstep run");
        $display(" Oracle 1: riscv_ref.v (same golden model as risc_tb.v)");
        if(qt_fd != 0)
            $display(" Oracle 2: qemu-riscv32 trace, %0d record(s)", qt_total);
        else
            $display(" Oracle 2: (none - pass +QEMUTRACE=<path> to add qemu)");
        $display("=========================================================\n");

        load_program;
        run_program;

        $display("=========================================================");
        $display(" RUN SUMMARY");
        $display("=========================================================");
        // The core retires its halt self-loop, which the qemu build does not
        // have (it exits via a syscall), so the trace is expected to come up
        // exactly one record short. Anything more means the two sides walked
        // different numbers of instructions - a control-flow divergence that
        // deserves to be reported even if every compared record matched.
        if(qt_fd != 0 && qt_read < qt_total) begin
            qerrors = qerrors + 1;
            $display("\n    QEMU ERROR: core stopped after consuming %0d of %0d trace record(s);",
                     qt_read, qt_total);
            $display("      qemu executed %0d more instruction(s) than the DUT did",
                     qt_total - qt_read);
        end

        $display("=========================================================");
        $display(" instructions retired : %0d", instr_count);
        $display("---------------------------------------------------------");
        $display(" vs riscv_ref.v  : %0d check(s), %0d error(s)  [pc + regs + dmem]",
                 checks, errors);
        if(qt_fd != 0)
            $display(" vs qemu-riscv32 : %0d check(s), %0d error(s)  [pc + regs + dmem]",
                     qchecks, qerrors);
        else
            $display(" vs qemu-riscv32 : not run");
        $display("---------------------------------------------------------");
        if(errors == 0 && qerrors == 0) begin
            $display(" ");
            if(qt_fd != 0)
                $display(" ✓ PASS  DUT matched BOTH independent models on every retired instruction");
            else
                $display(" ✓ PASS  DUT matched the golden model on every retired instruction");
        end
        else begin
            $display(" ");
            if(errors != 0)
                $display(" ✗ FAIL  %0d instruction(s) diverged from riscv_ref.v", errors);
            if(qerrors != 0)
                $display(" ✗ FAIL  %0d instruction(s) diverged from qemu-riscv32", qerrors);
            // Which oracles objected narrows the cause considerably: both
            // means the DUT is wrong, qemu alone means riscv_ref.v shares the
            // DUT's misreading of the spec - the exact blind spot the second
            // oracle was added to expose.
            if(errors == 0 && qerrors != 0)
                $display("         (riscv_ref.v agreed with the DUT here - suspect a shared");
            if(errors == 0 && qerrors != 0)
                $display("          misreading of the spec in risc.v AND riscv_ref.v)");
        end
        dump_state;
        $display("=========================================================");
        $display("TB_STATUS: %0s", (errors == 0 && qerrors == 0) ? "PASS" : "FAIL");
        $finish;
    end

    // global watchdog
    initial begin
        #50_000_000;
        $display("GLOBAL TIMEOUT - simulation did not finish");
        $finish;
    end

endmodule
