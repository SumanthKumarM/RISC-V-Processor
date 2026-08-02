//=============================================================================
// Self-checking testbench for the RV32I + RV32M multi-cycle core.
//
// Strategy: lockstep comparison against riscv_ref, a behavioural golden model.
// After EVERY instruction retires, the complete architectural state - pc, all
// 32 registers and all 64 data-memory words - is compared. Any divergence is
// reported with the offending instruction, its pc and a decoded mnemonic.
//
// Because the check is on full state after every instruction, a test does not
// need hand-computed expected values: it only needs to be an interesting
// program. That is what makes the random tests worth anything.
//
// Test flow for one "batch":
//    reset -> stream program into imem (and mirror into the model)
//          -> release core, seed registers/memory identically on both sides
//          -> retire prog_len instructions, comparing after each
//
// Every batch gets a `beq x0,x0,0` appended as a halt. If a test's control
// flow reaches it early, both the DUT and the model spin on the same pc, so
// the remaining comparisons stay valid instead of running off into garbage.
//=============================================================================
`timescale 1ns/1ps

module risc_tb;

    // Random seed, overridable from the Makefile: make SEED=1234
    parameter SEED = 20260801;

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

    // FSM state that marks a completed instruction (controller.call_nxt_pc)
    localparam S_CALL_NXT_PC = 3'b110;

    // opcodes
    localparam OP      = 7'b0110011;
    localparam OPIMM   = 7'b0010011;
    localparam LOAD    = 7'b0000011;
    localparam STORE   = 7'b0100011;
    localparam BRANCH  = 7'b1100011;
    localparam JAL_OP  = 7'b1101111;
    localparam JALR_OP = 7'b1100111;
    localparam LUI_OP  = 7'b0110111;
    localparam AUIPC   = 7'b0010111;

    //-------------------------------------------------------------------------
    // bookkeeping
    //-------------------------------------------------------------------------
    reg [31:0] prog [0:255];
    integer    prog_len;

    reg [31:0] seed_reg [0:31];
    reg [31:0] seed_mem [0:63];

    integer errors, checks, instr_count, batches;
    integer max_report;
    integer rseed;
    integer gi, gj, gt;

    //-------------------------------------------------------------------------
    // qemu oracle plumbing (optional second oracle, per batch)
    //
    //   +QEMU_DUMP=<dir>       dump each batch's program+seeds to
    //                          <dir>/batch_NNN.dump for offline processing by
    //                          qemu/tb_batch_gen.py + qemu/qemu_trace.py into
    //                          <dir>/batch_NNN.qtrace
    //   +QEMU_TRACE_DIR=<dir>  compare each batch against <dir>/batch_NNN.qtrace,
    //                          in addition to the riscv_ref.v check every batch
    //                          already gets
    //
    // A batch is not a standalone program (it starts from an arbitrary seeded
    // register file / data memory, not a zero reset), so unlike asm/*.s there
    // is no single qemu ELF to run once. Instead risc_tb.v itself is run
    // TWICE with the same SEED: once to dump the deterministically-generated
    // batches, once more to compare against the traces produced from those
    // dumps. Batch numbering (the `batches` counter) lines up between the two
    // runs because both walk the exact same sequence of run_batch calls.
    //-------------------------------------------------------------------------
    reg [8*256:1] qdump_dir, qtrace_dir;
    integer       qdump_en, qtrace_en;
    integer       qerrors, qchecks;

    integer       qt_fd, qt_total, qt_read;
    reg [31:0]    qt_pc;
    reg [31:0]    qt_regs [0:31];
    reg [31:0]    qt_dmem [0:63];

    //=========================================================================
    // instruction encoders
    //=========================================================================
    function [31:0] rtype;
        input [6:0] f7; input [4:0] rs2; input [4:0] rs1;
        input [2:0] f3; input [4:0] rd;  input [6:0] op;
        begin rtype = {f7,rs2,rs1,f3,rd,op}; end
    endfunction

    function [31:0] itype;
        input [11:0] imm; input [4:0] rs1; input [2:0] f3;
        input [4:0]  rd;  input [6:0] op;
        begin itype = {imm,rs1,f3,rd,op}; end
    endfunction

    function [31:0] stype;
        input [11:0] imm; input [4:0] rs2; input [4:0] rs1;
        input [2:0]  f3;  input [6:0] op;
        begin stype = {imm[11:5],rs2,rs1,f3,imm[4:0],op}; end
    endfunction

    function [31:0] btype;
        input [12:0] imm; input [4:0] rs2; input [4:0] rs1;
        input [2:0]  f3;  input [6:0] op;
        begin btype = {imm[12],imm[10:5],rs2,rs1,f3,imm[4:1],imm[11],op}; end
    endfunction

    function [31:0] utype;
        input [19:0] imm; input [4:0] rd; input [6:0] op;
        begin utype = {imm,rd,op}; end
    endfunction

    function [31:0] jtype;
        input [20:0] imm; input [4:0] rd; input [6:0] op;
        begin jtype = {imm[20],imm[10:1],imm[11],imm[19:12],rd,op}; end
    endfunction

    //=========================================================================
    // mnemonic decode, purely for readable failure messages
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
    // program / seed construction helpers
    //=========================================================================
    task clear_prog;
        begin prog_len = 0; end
    endtask

    task emit;
        input [31:0] w;
        begin
            if(prog_len < 255) begin
                prog[prog_len] = w;
                prog_len       = prog_len + 1;
            end
        end
    endtask

    task clear_seed;
        integer i;
        begin
            for(i=0;i<32;i=i+1) seed_reg[i] = 32'd0;
            for(i=0;i<64;i=i+1) seed_mem[i] = 32'd0;
        end
    endtask

    // A spread of values chosen to stress sign boundaries, carries and shifts.
    task set_corner_regs;
        begin
            clear_seed;
            seed_reg[1]  = 32'h00000000;
            seed_reg[2]  = 32'h00000001;
            seed_reg[3]  = 32'hFFFFFFFF; // -1
            seed_reg[4]  = 32'h00000002;
            seed_reg[5]  = 32'h7FFFFFFF; // INT_MAX
            seed_reg[6]  = 32'h80000000; // INT_MIN
            seed_reg[7]  = 32'h55555555;
            seed_reg[8]  = 32'hAAAAAAAA;
            seed_reg[9]  = 32'd31;
            seed_reg[10] = 32'd32;       // shift amount that must wrap to 0
            seed_reg[11] = 32'd33;       // shift amount that must wrap to 1
            seed_reg[12] = 32'hFFFFFFFE; // -2
        end
    endtask

    //=========================================================================
    // execution control
    //=========================================================================
    // Wait until the instruction currently in flight has fully retired.
    // The posedge leaving call_nxt_pc commits the PC update; register and
    // memory writes have already landed by then.
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
                #1;   // let combinational logic settle before sampling state
            end
        end
    endtask

    task compare;
        input [8*24:1] tname;
        input integer  idx;
        input [31:0]   ir_pc;
        input [31:0]   ir;
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
                    $display("\n    ERROR #%0d @ [%0s] instr %0d", errors, tname, idx);
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
    // qemu oracle: dump a batch's program + seeds for offline processing
    //
    // treloc/dreloc are bitmasks over registers 1..31: treloc marks registers
    // that hold a small offset from the BATCH'S OWN pc=0 start (typically a
    // JALR target computed against the core's zero-based address space -
    // e.g. `jalr x9,0(x9)` with x9 seeded to 16, meaning "16 bytes into this
    // program"), which tb_batch_gen.py must turn into __trace_start+offset
    // under qemu. dreloc marks registers that hold an offset into the 64-word
    // data window (the load/store base register, always x1 by convention in
    // this testbench), which must become DATA_BASE+offset under qemu. Most
    // batches need neither (0,0): ALU/branch/jump-immediate streams are
    // relocation-free because every address in them is either pc-relative or
    // computed at runtime by the DUT itself.
    //=========================================================================
    task dump_batch;
        input integer bnum;
        input [31:0]  treloc;
        input [31:0]  dreloc;
        integer fd, i;
        reg [8*300:1] path;
        begin
            $sformat(path, "%0s/batch_%03d.dump", qdump_dir, bnum);
            fd = $fopen(path, "w");
            if(fd == 0) begin
                $display("ERROR: could not open dump file '%0s'", path);
                $finish;
            end
            $fdisplay(fd, "%0d", prog_len);
            for(i=0;i<prog_len;i=i+1) $fdisplay(fd, "%08x", prog[i]);
            for(i=0;i<32;i=i+1)       $fdisplay(fd, "%08x", seed_reg[i]);
            for(i=0;i<64;i=i+1)       $fdisplay(fd, "%08x", seed_mem[i]);
            $fdisplay(fd, "%08x", treloc);
            $fdisplay(fd, "%08x", dreloc);
            $fclose(fd);
        end
    endtask

    //=========================================================================
    // qemu oracle: open/consume/close the per-batch trace produced offline
    // (same record format and shadow-memory folding as risc_tb_asm.v's qt_*
    // tasks; duplicated rather than shared because the two testbenches are
    // compiled as separate top-level modules)
    //=========================================================================
    task qt_open_batch;
        input integer bnum;
        integer rc, i;
        reg [8*300:1] path;
        begin
            qt_fd = 0; qt_total = 0; qt_read = 0;
            // The shadow memory must start from the SAME pre-seeded baseline
            // DUT.data_memory and REF.dmem get (see run_batch step 3 below),
            // not zero: a batch's data memory is seeded before execution, and
            // words the program never stores to stay at that seed forever -
            // qemu's own memory was seeded identically by tb_batch_gen.py's
            // prologue, but the trace itself only ever records STORES, so
            // without this the shadow would incorrectly start blank.
            for(i=0;i<64;i=i+1) qt_dmem[i] = seed_mem[i];
            if(qtrace_en) begin
                $sformat(path, "%0s/batch_%03d.qtrace", qtrace_dir, bnum);
                qt_fd = $fopen(path, "r");
                if(qt_fd == 0) begin
                    $display("ERROR: could not open QEMUTRACE '%0s'", path);
                    $finish;
                end
                rc = $fscanf(qt_fd, "%d", qt_total);
                if(rc != 1 || qt_total < 0) begin
                    $display("ERROR: QEMUTRACE '%0s' missing record-count header", path);
                    $finish;
                end
            end
        end
    endtask

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

    // Excludes the batch's appended halt self-loop: that instruction has no
    // qemu counterpart (a real self-loop would hang qemu forever), so the
    // caller must not invoke this for the final (prog_len-1) instruction.
    task qt_step_batch;
        input [8*24:1] tname;
        input integer  idx;
        input [31:0]   ir_pc;
        input [31:0]   ir;
        integer i, rc, nst, s;
        reg [31:0] a, sz, d;
        reg        bad;
        begin
            if(qt_fd != 0 && qt_read < qt_total) begin
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
                        $display("\n    QEMU ERROR #%0d @ [%0s] instr %0d", qerrors, tname, idx);
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
    endtask

    task qt_close_batch;
        input [8*24:1] tname;
        input integer  bnum;
        begin
            if(qtrace_en) begin
                if(qt_read != qt_total) begin
                    qerrors = qerrors + 1;
                    $display("\n    QEMU ERROR @ [%0s] batch %0d: consumed %0d of %0d trace record(s)",
                             tname, bnum, qt_read, qt_total);
                end
                if(qt_fd != 0) $fclose(qt_fd);
            end
        end
    endtask

    task run_batch;
        input [8*24:1] tname;
        input [31:0]   treloc;   // qemu oracle: registers needing +__trace_start reloc
        input [31:0]   dreloc;   // qemu oracle: registers needing +DATA_BASE reloc
        integer i;
        reg     timeout, bail;
        reg [31:0] ir_pc, ir;
        begin
            emit(btype(13'd0, 5'd0, 5'd0, 3'b000, BRANCH)); // halt: beq x0,x0,0
            batches = batches + 1;
            $display("  [batch %3d] %-16s : %3d instructions", batches, tname, prog_len-1);

            if(qdump_en) dump_batch(batches, treloc, dreloc);
            qt_open_batch(batches);

            // ---- 1. reset the DUT (clears pc, regs, dmem and the load pointer)
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

            // ---- 3. release the core, then seed both sides identically
            @(negedge clk);
            rst = 1'b0;
            REF.reset_state;
            for(i=0;i<32;i=i+1) begin
                DUT.register_file.mem[i] = seed_reg[i];
                REF.regs[i]              = seed_reg[i];
            end
            DUT.register_file.mem[0] = 32'd0;   // x0 stays hardwired
            REF.regs[0]              = 32'd0;
            for(i=0;i<64;i=i+1) begin
                DUT.data_memory.mem[i] = seed_mem[i];
                REF.dmem[i]            = seed_mem[i];
            end

            // ---- 4. lockstep execution
            bail = 1'b0;
            for(i=0;i<prog_len;i=i+1) begin
                if(!bail) begin
                    ir_pc = REF.pc;
                    ir    = REF.imem[REF.pc[9:2]];
                    wait_retire(timeout);
                    if(timeout) begin
                        errors = errors + 1;
                        if(errors <= max_report)
                            $display("    [%3d] TIMEOUT at pc=%h : %s", i, ir_pc, mnem(ir));
                        bail = 1'b1;
                    end
                    else begin
                        REF.step;
                        instr_count = instr_count + 1;
                        compare(tname, i, ir_pc, ir);
                        // the appended halt (index prog_len-1) has no qemu counterpart
                        if(i < prog_len-1) qt_step_batch(tname, i, ir_pc, ir);
                        // show progress: print dots every 10 instructions, numbers every 50
                        if((i % 50) == 49)
                            $write("[%3d]", i+1);
                        else if((i % 10) == 9)
                            $write(".");
                        $fflush();
                    end
                end
            end
            qt_close_batch(tname, batches);
            if(prog_len > 1) $display(" done");
            else $display("");
        end
    endtask

    //=========================================================================
    // Directed tests
    //=========================================================================

    // Every RV32I R-type op against a 12x12 matrix of corner operands.
    task test_rtype;
        reg [6:0] f7; reg [2:0] f3;
        begin
            for(gt=0;gt<10;gt=gt+1) begin
                case(gt)
                    0: begin f7=7'b0000000; f3=3'b000; end // ADD
                    1: begin f7=7'b0100000; f3=3'b000; end // SUB
                    2: begin f7=7'b0000000; f3=3'b001; end // SLL
                    3: begin f7=7'b0000000; f3=3'b010; end // SLT
                    4: begin f7=7'b0000000; f3=3'b011; end // SLTU
                    5: begin f7=7'b0000000; f3=3'b100; end // XOR
                    6: begin f7=7'b0000000; f3=3'b101; end // SRL
                    7: begin f7=7'b0100000; f3=3'b101; end // SRA
                    8: begin f7=7'b0000000; f3=3'b110; end // OR
                    9: begin f7=7'b0000000; f3=3'b111; end // AND
                endcase
                clear_prog; set_corner_regs;
                for(gi=1;gi<=12;gi=gi+1)
                    for(gj=1;gj<=12;gj=gj+1)
                        emit(rtype(f7, gj[4:0], gi[4:0], f3, 5'd20, OP));
                run_batch("R-type", 32'h0, 32'h0);
            end
        end
    endtask

    // All eight RV32M ops against the same matrix. This covers div-by-zero
    // (x1 = 0 as divisor) and the -2^31 / -1 overflow case (x6 / x3).
    task test_mext;
        begin
            for(gt=0;gt<8;gt=gt+1) begin
                clear_prog; set_corner_regs;
                for(gi=1;gi<=12;gi=gi+1)
                    for(gj=1;gj<=12;gj=gj+1)
                        emit(rtype(7'b0000001, gj[4:0], gi[4:0], gt[2:0], 5'd20, OP));
                run_batch("RV32M", 32'h0, 32'h0);
            end
        end
    endtask

    // I-type ALU ops against corner registers x corner immediates.
    task test_itype;
        reg [2:0]  f3;
        reg [11:0] imm;
        begin
            for(gt=0;gt<6;gt=gt+1) begin
                case(gt)
                    0: f3 = 3'b000; // ADDI
                    1: f3 = 3'b010; // SLTI
                    2: f3 = 3'b011; // SLTIU
                    3: f3 = 3'b100; // XORI
                    4: f3 = 3'b110; // ORI
                    5: f3 = 3'b111; // ANDI
                endcase
                clear_prog; set_corner_regs;
                for(gi=1;gi<=12;gi=gi+1)
                    for(gj=0;gj<8;gj=gj+1) begin
                        case(gj)
                            0: imm = 12'h000;
                            1: imm = 12'h001;
                            2: imm = 12'hFFF; // -1
                            3: imm = 12'h7FF; // +2047
                            4: imm = 12'h800; // -2048
                            5: imm = 12'h555;
                            6: imm = 12'hAAA;
                            7: imm = 12'h064; // +100
                        endcase
                        emit(itype(imm, gi[4:0], f3, 5'd20, OPIMM));
                    end
                run_batch("I-type", 32'h0, 32'h0);
            end
        end
    endtask

    // SLLI / SRLI / SRAI over every shift amount 0..31.
    // SRAI carries funct7=0100000 in the same field as the shamt, which is the
    // exact case that used to corrupt the shift amount.
    task test_shift_imm;
        reg [2:0] f3; reg [6:0] f7;
        begin
            for(gt=0;gt<3;gt=gt+1) begin
                case(gt)
                    0: begin f3=3'b001; f7=7'b0000000; end // SLLI
                    1: begin f3=3'b101; f7=7'b0000000; end // SRLI
                    2: begin f3=3'b101; f7=7'b0100000; end // SRAI
                endcase
                clear_prog; set_corner_regs;
                for(gi=1;gi<=6;gi=gi+1)
                    for(gj=0;gj<32;gj=gj+1)
                        emit(itype({f7,gj[4:0]}, gi[4:0], f3, 5'd20, OPIMM));
                run_batch("shift-imm", 32'h0, 32'h0);
            end
        end
    endtask

    // All six branches, taken and not-taken, over a corner-operand matrix.
    // A wrong branch direction shows up immediately as a pc mismatch.
    task test_branch;
        reg [2:0] f3;
        begin
            for(gt=0;gt<6;gt=gt+1) begin
                case(gt)
                    0: f3 = 3'b000; // BEQ
                    1: f3 = 3'b001; // BNE
                    2: f3 = 3'b100; // BLT
                    3: f3 = 3'b101; // BGE
                    4: f3 = 3'b110; // BLTU
                    5: f3 = 3'b111; // BGEU
                endcase
                clear_prog; set_corner_regs;
                for(gi=1;gi<=8;gi=gi+1)
                    for(gj=1;gj<=8;gj=gj+1) begin
                        emit(btype(13'd8, gj[4:0], gi[4:0], f3, BRANCH));
                        emit(itype(12'd1, 5'd0, 3'b000, 5'd20, OPIMM)); // skipped if taken
                    end
                run_batch("branch", 32'h0, 32'h0);
            end
        end
    endtask

    // A backward branch forming a real counted loop.
    task test_loop;
        begin
            clear_prog; clear_seed;
            seed_reg[1] = 32'd0;   // counter
            seed_reg[2] = 32'd10;  // limit
            seed_reg[3] = 32'd0;   // accumulator
            emit(itype(12'd1, 5'd1, 3'b000, 5'd1, OPIMM));      // 0: addi x1,x1,1
            emit(rtype(7'd0, 5'd1, 5'd3, 3'b000, 5'd3, OP));    // 1: add  x3,x3,x1
            emit(btype(-13'sd8, 5'd2, 5'd1, 3'b001, BRANCH));   // 2: bne  x1,x2,-8
            emit(itype(12'd99, 5'd0, 3'b000, 5'd4, OPIMM));     // 3: addi x4,x0,99
            // 10 iterations x 3 instructions + tail; padded by the halt
            prog_len = prog_len; // keep as-is
            run_batch("loop", 32'h0, 32'h0);
        end
    endtask

    // JAL / JALR: return addresses, targets, low-bit clearing, x0 as rd,
    // and the rd == rs1 case where the link write must not disturb the target.
    task test_jumps;
        begin
            // --- basic jal / jalr chain ---
            clear_prog; clear_seed;
            emit(jtype(21'd8,  5'd1, JAL_OP));                  // 0: jal x1,+8   -> 8
            emit(itype(12'd1,  5'd0, 3'b000, 5'd20, OPIMM));    // 1: skipped
            emit(utype(20'd0,  5'd2, AUIPC));                   // 2: auipc x2,0  -> x2=8
            emit(itype(12'd20, 5'd2, 3'b000, 5'd2, OPIMM));     // 3: x2 = 28
            emit(itype(12'd0,  5'd2, 3'b000, 5'd3, JALR_OP));   // 4: jalr x3,0(x2) -> 28
            emit(itype(12'd2,  5'd0, 3'b000, 5'd20, OPIMM));    // 5: skipped
            emit(itype(12'd3,  5'd0, 3'b000, 5'd20, OPIMM));    // 6: skipped
            emit(itype(12'd42, 5'd0, 3'b000, 5'd4, OPIMM));     // 7: addi x4,x0,42
            run_batch("jump-basic", 32'h0, 32'h0);

            // --- jalr with an odd target: bit 0 of the result must be cleared ---
            clear_prog; clear_seed;
            seed_reg[5] = 32'd13;                               // odd base
            emit(itype(12'd3, 5'd5, 3'b000, 5'd6, JALR_OP));    // 0: 13+3=16 -> pc=16
            emit(itype(12'd1, 5'd0, 3'b000, 5'd20, OPIMM));     // 1
            emit(itype(12'd2, 5'd0, 3'b000, 5'd20, OPIMM));     // 2
            emit(itype(12'd3, 5'd0, 3'b000, 5'd20, OPIMM));     // 3
            emit(itype(12'd7, 5'd0, 3'b000, 5'd7, OPIMM));      // 4: at byte 16
            // x5 holds a literal offset into this batch's own instruction
            // stream (a core-address JALR target) - qemu oracle needs it
            // relocated to __trace_start+13, hence treloc bit 5.
            run_batch("jalr-odd", 32'h0000_0020, 32'h0);

            // --- jal/jalr writing x0: link value must be discarded ---
            clear_prog; clear_seed;
            seed_reg[5] = 32'd12;
            emit(jtype(21'd8, 5'd0, JAL_OP));                   // 0: jal x0,+8
            emit(itype(12'd1, 5'd0, 3'b000, 5'd20, OPIMM));     // 1: skipped
            emit(itype(12'd0, 5'd5, 3'b000, 5'd0, JALR_OP));    // 2: jalr x0,0(x5) -> 12
            emit(itype(12'd9, 5'd0, 3'b000, 5'd8, OPIMM));      // 3: at byte 12
            run_batch("jump-x0", 32'h0000_0020, 32'h0);         // x5 again holds a JALR target

            // --- jalr where rd == rs1 ---
            // The target must be computed from the ORIGINAL rs1, even though
            // the same register is overwritten with the return address.
            clear_prog; clear_seed;
            seed_reg[9] = 32'd16;
            emit(itype(12'd0, 5'd9, 3'b000, 5'd9, JALR_OP));    // 0: jalr x9,0(x9) -> 16
            emit(itype(12'd1, 5'd0, 3'b000, 5'd20, OPIMM));     // 1
            emit(itype(12'd2, 5'd0, 3'b000, 5'd20, OPIMM));     // 2
            emit(itype(12'd3, 5'd0, 3'b000, 5'd20, OPIMM));     // 3
            emit(itype(12'd5, 5'd0, 3'b000, 5'd10, OPIMM));     // 4: at byte 16
            run_batch("jalr-rd-eq-rs1", 32'h0000_0200, 32'h0);  // x9 holds the JALR target

            // --- negative jal offset (backward jump) ---
            clear_prog; clear_seed;
            emit(jtype(21'd12, 5'd1, JAL_OP));                  // 0: jal x1,+12 -> 12
            emit(itype(12'd1, 5'd0, 3'b000, 5'd20, OPIMM));     // 1
            emit(itype(12'd7, 5'd0, 3'b000, 5'd11, OPIMM));     // 2: landing pad
            emit(jtype(-21'sd4, 5'd2, JAL_OP));                 // 3: jal x2,-4 -> 8
            run_batch("jal-backward", 32'h0, 32'h0);
        end
    endtask

    // LUI / AUIPC with corner immediates.
    task test_upper;
        reg [19:0] u;
        begin
            clear_prog; clear_seed;
            for(gj=0;gj<8;gj=gj+1) begin
                case(gj)
                    0: u = 20'h00000;
                    1: u = 20'h00001;
                    2: u = 20'hFFFFF;
                    3: u = 20'h80000;
                    4: u = 20'h7FFFF;
                    5: u = 20'h55555;
                    6: u = 20'hAAAAA;
                    7: u = 20'hABCDE;
                endcase
                emit(utype(u, 5'd20, LUI_OP));
                emit(utype(u, 5'd21, AUIPC));
            end
            run_batch("lui-auipc", 32'h0, 32'h0);
        end
    endtask

    // Every load/store width at every byte offset, checking sign vs zero
    // extension and that partial stores leave neighbouring bytes alone.
    task test_load_store;
        begin
            // byte stores at all four offsets into a known pattern
            clear_prog; clear_seed;
            seed_reg[1] = 32'd0;                 // base
            seed_reg[2] = 32'h000000AB;
            seed_reg[3] = 32'h0000CDEF;
            seed_reg[4] = 32'h89ABCDEF;
            for(gi=0;gi<64;gi=gi+1) seed_mem[gi] = 32'h11223344;
            for(gj=0;gj<4;gj=gj+1) begin
                emit(stype(gj[11:0], 5'd2, 5'd1, 3'b000, STORE));      // sb
                emit(itype(gj[11:0], 5'd1, 3'b000, 5'd20, LOAD));      // lb
                emit(itype(gj[11:0], 5'd1, 3'b100, 5'd21, LOAD));      // lbu
            end
            emit(stype(12'd8,  5'd3, 5'd1, 3'b001, STORE));            // sh  @8
            emit(stype(12'd10, 5'd3, 5'd1, 3'b001, STORE));            // sh  @10
            emit(itype(12'd8,  5'd1, 3'b001, 5'd22, LOAD));            // lh  @8
            emit(itype(12'd10, 5'd1, 3'b001, 5'd23, LOAD));            // lh  @10
            emit(itype(12'd8,  5'd1, 3'b101, 5'd24, LOAD));            // lhu @8
            emit(itype(12'd10, 5'd1, 3'b101, 5'd25, LOAD));            // lhu @10
            emit(stype(12'd16, 5'd4, 5'd1, 3'b010, STORE));            // sw  @16
            emit(itype(12'd16, 5'd1, 3'b010, 5'd26, LOAD));            // lw  @16
            // x1 is the load/store base register (by this testbench's
            // convention); qemu oracle needs it relocated to DATA_BASE+offset.
            run_batch("ld-st-basic", 32'h0, 32'h0000_0002);

            // sign-extension corners: 0x80 / 0x7F bytes and 0x8000 / 0x7FFF halves
            clear_prog; clear_seed;
            seed_reg[1] = 32'd0;
            seed_reg[2] = 32'h00000080;
            seed_reg[3] = 32'h0000007F;
            seed_reg[4] = 32'h00008000;
            seed_reg[5] = 32'h00007FFF;
            emit(stype(12'd0,  5'd2, 5'd1, 3'b000, STORE));   // sb 0x80
            emit(itype(12'd0,  5'd1, 3'b000, 5'd20, LOAD));   // lb  -> 0xFFFFFF80
            emit(itype(12'd0,  5'd1, 3'b100, 5'd21, LOAD));   // lbu -> 0x00000080
            emit(stype(12'd1,  5'd3, 5'd1, 3'b000, STORE));   // sb 0x7F
            emit(itype(12'd1,  5'd1, 3'b000, 5'd22, LOAD));   // lb  -> 0x0000007F
            emit(stype(12'd4,  5'd4, 5'd1, 3'b001, STORE));   // sh 0x8000
            emit(itype(12'd4,  5'd1, 3'b001, 5'd23, LOAD));   // lh  -> 0xFFFF8000
            emit(itype(12'd4,  5'd1, 3'b101, 5'd24, LOAD));   // lhu -> 0x00008000
            emit(stype(12'd6,  5'd5, 5'd1, 3'b001, STORE));   // sh 0x7FFF
            emit(itype(12'd6,  5'd1, 3'b001, 5'd25, LOAD));   // lh  -> 0x00007FFF
            run_batch("ld-st-signext", 32'h0, 32'h0000_0002);

            // negative store/load offsets, exercising immS sign extension
            clear_prog; clear_seed;
            seed_reg[1] = 32'd64;
            seed_reg[2] = 32'hDEADBEEF;
            emit(stype(-12'sd4, 5'd2, 5'd1, 3'b010, STORE));  // sw x2,-4(x1) -> byte 60
            emit(itype(-12'sd4, 5'd1, 3'b010, 5'd20, LOAD));  // lw x20,-4(x1)
            emit(stype(-12'sd64, 5'd2, 5'd1, 3'b010, STORE)); // sw -> byte 0
            emit(itype(-12'sd64, 5'd1, 3'b010, 5'd21, LOAD));
            run_batch("ld-st-negoff", 32'h0, 32'h0000_0002);
        end
    endtask

    // x0 must read as zero and absorb writes from every instruction class.
    task test_x0;
        begin
            clear_prog; set_corner_regs;
            emit(itype(12'd5,  5'd0, 3'b000, 5'd0, OPIMM));            // addi x0,x0,5
            emit(rtype(7'd0, 5'd5, 5'd6, 3'b000, 5'd0, OP));           // add  x0,x6,x5
            emit(utype(20'hFFFFF, 5'd0, LUI_OP));                      // lui  x0
            emit(utype(20'h00001, 5'd0, AUIPC));                       // auipc x0
            emit(rtype(7'b0000001, 5'd5, 5'd6, 3'b000, 5'd0, OP));     // mul  x0
            emit(itype(12'd0, 5'd1, 3'b010, 5'd0, LOAD));              // lw   x0,0(x1)
            emit(rtype(7'd0, 5'd0, 5'd0, 3'b000, 5'd20, OP));          // add  x20,x0,x0
            run_batch("x0-behaviour", 32'h0, 32'h0000_0002);           // x1 is the lw base
        end
    endtask

    //=========================================================================
    // Randomised tests
    //=========================================================================
    // Random straight-line streams of ALU / M / load / store instructions with
    // randomised register and memory seeds. Control flow is excluded here so
    // the stream is guaranteed to make progress; branches and jumps are
    // covered by the directed tests above.
    task test_random;
        input integer n_batches;
        input integer n_instr;
        integer bi, i, sel, vsel;
        reg [4:0]  rs1a, rs2a, rd;
        reg [2:0]  f3;
        reg [6:0]  f7;
        reg [11:0] imm;
        reg [31:0] v;
        begin
            for(bi=0; bi<n_batches; bi=bi+1) begin
                clear_prog; clear_seed;

                // seed registers with a mix of corner and random values
                for(i=1;i<32;i=i+1) begin
                    v    = {$random(rseed)};
                    vsel = {$random(rseed)} % 10;
                    case(vsel)
                        0: seed_reg[i] = 32'h00000000;
                        1: seed_reg[i] = 32'hFFFFFFFF;
                        2: seed_reg[i] = 32'h80000000;
                        3: seed_reg[i] = 32'h7FFFFFFF;
                        4: seed_reg[i] = 32'h00000001;
                        default: seed_reg[i] = v;
                    endcase
                end
                seed_reg[1] = 32'd0;   // dedicated base register for load/store
                for(i=0;i<64;i=i+1) seed_mem[i] = {$random(rseed)};

                for(i=0;i<n_instr;i=i+1) begin
                    rs1a = {$random(rseed)} % 32;
                    rs2a = {$random(rseed)} % 32;
                    rd   = {$random(rseed)} % 32;
                    imm  = {$random(rseed)};
                    sel  = {$random(rseed)} % 100;

                    // x1 is reserved as the load/store base and must stay 0.
                    // If a random instruction clobbered it the generated
                    // addresses would drift off their natural alignment, and
                    // misaligned access is explicitly undefined in this core -
                    // testing it would be testing nothing.
                    //
                    // x1 is ALSO excluded as a general ALU source (rs1a/rs2a)
                    // here, not just as a destination: the qemu oracle (see
                    // qemu/tb_batch_gen.py) reproduces x1's role as a load/
                    // store base by giving it a DIFFERENT absolute value than
                    // the DUT (DATA_BASE+0 under qemu vs plain 0 on the core -
                    // qemu cannot map address 0). Reading x1 into an ordinary
                    // ADD/AND/MUL/etc would bake that qemu-only offset into
                    // whatever register receives the result, and for anything
                    // other than pure +/- it is not even a constant offset
                    // (e.g. AND with a DATA_BASE-shifted operand does not
                    // differ from the DUT's by a fixed amount at all) - not
                    // reconcilable after the fact, so the only sound fix is to
                    // never let x1 enter general ALU arithmetic in the first
                    // place. This does not reduce ISA coverage: x1 already
                    // gets full load/store coverage as the dedicated base.
                    if(rd == 5'd1) rd = 5'd2;
                    if(rs1a == 5'd1) rs1a = 5'd2;
                    if(rs2a == 5'd1) rs2a = 5'd2;

                    if(sel < 40) begin
                        // RV32I R-type
                        f3 = {$random(rseed)} % 8;
                        f7 = 7'b0000000;
                        if((f3 == 3'b000) || (f3 == 3'b101))
                            if({$random(rseed)} % 2) f7 = 7'b0100000;
                        emit(rtype(f7, rs2a, rs1a, f3, rd, OP));
                    end
                    else if(sel < 62) begin
                        // RV32M
                        f3 = {$random(rseed)} % 8;
                        emit(rtype(7'b0000001, rs2a, rs1a, f3, rd, OP));
                    end
                    else if(sel < 87) begin
                        // RV32I I-type (shifts get a legal shamt + funct7)
                        f3 = {$random(rseed)} % 8;
                        if(f3 == 3'b001)
                            imm = {7'b0000000, imm[4:0]};
                        else if(f3 == 3'b101)
                            imm = {({$random(rseed)}%2) ? 7'b0100000 : 7'b0000000, imm[4:0]};
                        emit(itype(imm, rs1a, f3, rd, OPIMM));
                    end
                    else begin
                        // aligned load / store within the 256-byte data memory
                        f3 = {$random(rseed)} % 3;             // 0=byte 1=half 2=word
                        imm = {$random(rseed)} % 256;
                        if(f3 == 3'b001) imm = imm & 12'hFFE;  // halfword aligned
                        if(f3 == 3'b010) imm = imm & 12'hFFC;  // word aligned
                        if({$random(rseed)} % 2)
                            emit(stype(imm, rs2a, 5'd1, f3, STORE));
                        else begin
                            // loads may also be the unsigned forms
                            if((f3 != 3'b010) && ({$random(rseed)} % 2))
                                f3 = f3 | 3'b100;
                            emit(itype(imm, 5'd1, f3, rd, LOAD));
                        end
                    end
                end
                run_batch("random", 32'h0, 32'h0000_0002);              // x1 is the load/store base
            end
        end
    endtask

    //=========================================================================
    // main
    //=========================================================================
    initial begin
`ifdef DUMP
        $dumpfile("risc_tb.vcd");
        $dumpvars(0, risc_tb);
`endif
        clk            = 1'b0;
        rst            = 1'b1;
        imem_load_en   = 1'b0;
        imem_load_data = 32'd0;
        prog_len       = 0;
        errors         = 0;
        checks         = 0;
        instr_count    = 0;
        batches        = 0;
        max_report     = 25;
        rseed          = SEED;
        qerrors        = 0;
        qchecks        = 0;
        qdump_en       = $value$plusargs("QEMU_DUMP=%s", qdump_dir);
        qtrace_en      = $value$plusargs("QEMU_TRACE_DIR=%s", qtrace_dir);

        clear_seed;

        $display("=========================================================");
        $display(" RV32I + RV32M core verification (lockstep vs reference)");
        $display(" Lockstep: full state (pc, regs, memory) checked after");
        $display(" every instruction execution against golden model");
        if(qdump_en)
            $display(" qemu oracle: dumping batches to %0s", qdump_dir);
        else if(qtrace_en)
            $display(" qemu oracle: comparing batches against traces in %0s", qtrace_dir);
        else
            $display(" qemu oracle: not run (see qemu/README.md, `make run_tb_qemu`)");
        $display("=========================================================\n");

        $display("DIRECTED TESTS:");
        test_rtype;
        $display("");
        test_itype;
        $display("");
        test_shift_imm;
        $display("");
        test_mext;
        $display("");
        test_upper;
        $display("");
        test_branch;
        $display("");
        test_loop;
        $display("");
        test_jumps;
        $display("");
        test_load_store;
        $display("");
        test_x0;

        $display("\nRANDOMISED TESTS:");
        test_random(10, 180);

        $display("=========================================================");
        $display(" TEST SUMMARY");
        $display("=========================================================");
        $display(" test batches         : %0d", batches);
        $display(" instructions retired : %0d", instr_count);
        $display(" vs riscv_ref.v       : %0d check(s), %0d error(s)  [pc + regs + dmem]", checks, errors);
        if(qtrace_en)
            $display(" vs qemu-riscv32      : %0d check(s), %0d error(s)  [pc + regs + dmem]", qchecks, qerrors);
        else if(!qdump_en)
            $display(" vs qemu-riscv32      : not run (`make run_tb_qemu`, see qemu/README.md)");
        if(errors == 0 && qerrors == 0) begin
            $display(" ");
            if(qtrace_en)
                $display(" ✓ PASS  All %0d instructions verified correctly vs BOTH oracles", instr_count);
            else
                $display(" ✓ PASS  All %0d instructions verified correctly", instr_count);
        end
        else begin
            $display(" ");
            if(errors != 0)
                $display(" ✗ FAIL  %0d instruction(s) diverged from riscv_ref.v", errors);
            if(qerrors != 0)
                $display(" ✗ FAIL  %0d instruction(s) diverged from qemu-riscv32", qerrors);
            $display("         See ERROR logs above for details");
        end
        $display("=========================================================");
        // Plain-ASCII marker for tooling (Makefile pass/fail check). Kept
        // separate from the pretty summary above so reformatting the human
        // output can never silently break the machine check.
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
