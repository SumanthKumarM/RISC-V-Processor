//=============================================================================
// Behavioural RV32I + RV32M reference model (golden model)
//
// This is an INDEPENDENT implementation of the ISA, written with the
// simulator's native operators (* / % >>> << etc). The DUT computes the same
// results with hand-built structural circuits (carry-lookahead adder, Booth
// recoder, non-restoring divider). Because the two arrive at the answer by
// completely different routes, a disagreement is meaningful evidence of a bug
// rather than the same mistake made twice.
//
// The model is architectural only: it has no notion of the DUT's FSM states or
// timing. The testbench drives it one instruction at a time (`step`) and
// compares the full architectural state (pc, 32 registers, data memory) after
// every instruction retires.
//
// Memory geometry deliberately mirrors the DUT so that address aliasing
// behaves identically:
//     imem : 256 words, indexed by pc[9:2]
//     dmem :  64 words, indexed by addr[7:2]
//=============================================================================
module riscv_ref;

    parameter IMEM_WORDS = 256;
    parameter DMEM_WORDS = 64;

    reg [31:0] imem [0:IMEM_WORDS-1];
    reg [31:0] dmem [0:DMEM_WORDS-1];
    reg [31:0] regs [0:31];
    reg [31:0] pc;
    reg        halted;   // set when an opcode outside RV32I/RV32M is fetched

    integer k;

    //-------------------------------------------------------------------------
    // RV32M helpers
    //-------------------------------------------------------------------------
    // Low 32 bits of the product: identical for signed and unsigned operands.
    function [31:0] f_mul;
        input [31:0] x, y;
        begin f_mul = x * y; end
    endfunction

    // High 32 bits, signed x signed. The 64-bit LHS forces both operands to be
    // sign-extended to 64 bits before the multiply.
    function [31:0] f_mulh;
        input [31:0] x, y;
        reg signed [63:0] p;
        reg signed [31:0] sx, sy;
        begin
            sx = x; sy = y;
            p  = sx * sy;
            f_mulh = p[63:32];
        end
    endfunction

    // High 32 bits, signed x unsigned.
    function [31:0] f_mulhsu;
        input [31:0] x, y;
        reg signed [63:0] p, sx, uy;
        begin
            sx = $signed({{32{x[31]}}, x});  // sign extended
            uy = $signed({32'd0, y});        // zero extended -> always positive
            p  = sx * uy;
            f_mulhsu = p[63:32];
        end
    endfunction

    // High 32 bits, unsigned x unsigned.
    function [31:0] f_mulhu;
        input [31:0] x, y;
        reg [63:0] p;
        begin
            p = x * y;
            f_mulhu = p[63:32];
        end
    endfunction

    // Signed divide. Spec: divide by zero yields all ones; the single
    // overflow case (-2^31 / -1) yields -2^31. Neither traps.
    function [31:0] f_div;
        input [31:0] x, y;
        reg signed [31:0] sx, sy;
        begin
            sx = x; sy = y;
            if(y == 32'd0)                                    f_div = 32'hFFFFFFFF;
            else if(x == 32'h80000000 && y == 32'hFFFFFFFF)   f_div = 32'h80000000;
            else                                              f_div = sx / sy;
        end
    endfunction

    // Signed remainder. Spec: divide by zero yields the dividend; the overflow
    // case yields 0. The sign of the result follows the dividend.
    function [31:0] f_rem;
        input [31:0] x, y;
        reg signed [31:0] sx, sy;
        begin
            sx = x; sy = y;
            if(y == 32'd0)                                    f_rem = x;
            else if(x == 32'h80000000 && y == 32'hFFFFFFFF)   f_rem = 32'd0;
            else                                              f_rem = sx % sy;
        end
    endfunction

    function [31:0] f_divu;
        input [31:0] x, y;
        begin
            if(y == 32'd0) f_divu = 32'hFFFFFFFF;
            else           f_divu = x / y;
        end
    endfunction

    function [31:0] f_remu;
        input [31:0] x, y;
        begin
            if(y == 32'd0) f_remu = x;
            else           f_remu = x % y;
        end
    endfunction

    //-------------------------------------------------------------------------
    // State control
    //-------------------------------------------------------------------------
    task reset_state;
        begin
            for(k=0;k<32;k=k+1)        regs[k] = 32'd0;
            for(k=0;k<DMEM_WORDS;k=k+1) dmem[k] = 32'd0;
            pc     = 32'd0;
            halted = 1'b0;
        end
    endtask

    task load_word;
        input integer idx;
        input [31:0]  w;
        begin imem[idx] = w; end
    endtask

    //-------------------------------------------------------------------------
    // Execute exactly one instruction
    //-------------------------------------------------------------------------
    task step;
        reg [31:0] ir;
        reg [6:0]  op, f7;
        reg [2:0]  f3;
        reg [4:0]  rd, rs1a, rs2a, sh;
        reg [31:0] a, b;
        reg [31:0] immI, immS, immB, immU, immJ;
        reg [31:0] res, addr, nextpc, w;
        reg [7:0]  lb;
        reg [15:0] lh;
        reg        wen, taken;
        begin
            ir   = imem[pc[9:2]];
            op   = ir[6:0];
            f3   = ir[14:12];
            f7   = ir[31:25];
            rd   = ir[11:7];
            rs1a = ir[19:15];
            rs2a = ir[24:20];
            sh   = ir[24:20];          // shamt for the immediate shifts
            a    = regs[rs1a];
            b    = regs[rs2a];

            immI = {{20{ir[31]}}, ir[31:20]};
            immS = {{20{ir[31]}}, ir[31:25], ir[11:7]};
            immB = {{19{ir[31]}}, ir[31], ir[7], ir[30:25], ir[11:8], 1'b0};
            immU = {ir[31:12], 12'd0};
            immJ = {{11{ir[31]}}, ir[31], ir[19:12], ir[20], ir[30:21], 1'b0};

            res    = 32'd0;
            wen    = 1'b0;
            taken  = 1'b0;
            nextpc = pc + 32'd4;

            case(op)
                //---------------------------------------------------- LUI ----
                7'b0110111 : begin res = immU;      wen = 1'b1; end
                //-------------------------------------------------- AUIPC ----
                7'b0010111 : begin res = pc + immU; wen = 1'b1; end
                //---------------------------------------------------- JAL ----
                7'b1101111 : begin
                    res    = pc + 32'd4;
                    wen    = 1'b1;
                    nextpc = pc + immJ;
                end
                //--------------------------------------------------- JALR ----
                // Target is computed from the ORIGINAL rs1, then the low bit is
                // cleared. rd is written with the return address afterwards, so
                // rd == rs1 must still jump using the old rs1 value.
                7'b1100111 : begin
                    nextpc = (a + immI) & ~32'd1;
                    res    = pc + 32'd4;
                    wen    = 1'b1;
                end
                //------------------------------------------------- BRANCH ----
                7'b1100011 : begin
                    case(f3)
                        3'b000 : taken = (a == b);                     // BEQ
                        3'b001 : taken = (a != b);                     // BNE
                        3'b100 : taken = ($signed(a) <  $signed(b));   // BLT
                        3'b101 : taken = ($signed(a) >= $signed(b));   // BGE
                        3'b110 : taken = (a <  b);                     // BLTU
                        3'b111 : taken = (a >= b);                     // BGEU
                        default: taken = 1'b0;
                    endcase
                    if(taken) nextpc = pc + immB;
                end
                //--------------------------------------------------- LOAD ----
                7'b0000011 : begin
                    addr = a + immI;
                    w    = dmem[addr[7:2]];
                    case(addr[1:0])
                        2'b00 : lb = w[7:0];
                        2'b01 : lb = w[15:8];
                        2'b10 : lb = w[23:16];
                        2'b11 : lb = w[31:24];
                    endcase
                    lh = addr[1] ? w[31:16] : w[15:0];
                    case(f3)
                        3'b000 : res = {{24{lb[7]}},  lb};   // LB
                        3'b001 : res = {{16{lh[15]}}, lh};   // LH
                        3'b010 : res = w;                    // LW
                        3'b100 : res = {24'd0, lb};          // LBU
                        3'b101 : res = {16'd0, lh};          // LHU
                        default: res = w;
                    endcase
                    wen = 1'b1;
                end
                //-------------------------------------------------- STORE ----
                7'b0100011 : begin
                    addr = a + immS;
                    w    = dmem[addr[7:2]];
                    case(f3)
                        3'b000 : begin // SB
                            case(addr[1:0])
                                2'b00 : w[7:0]   = b[7:0];
                                2'b01 : w[15:8]  = b[7:0];
                                2'b10 : w[23:16] = b[7:0];
                                2'b11 : w[31:24] = b[7:0];
                            endcase
                        end
                        3'b001 : begin // SH
                            if(addr[1]) w[31:16] = b[15:0];
                            else        w[15:0]  = b[15:0];
                        end
                        default : w = b;                     // SW
                    endcase
                    dmem[addr[7:2]] = w;
                end
                //------------------------------------------------- OP-IMM ----
                7'b0010011 : begin
                    case(f3)
                        3'b000 : res = a + immI;                                  // ADDI
                        3'b010 : res = ($signed(a) < $signed(immI)) ? 32'd1 : 32'd0; // SLTI
                        3'b011 : res = (a < immI) ? 32'd1 : 32'd0;                // SLTIU
                        3'b100 : res = a ^ immI;                                  // XORI
                        3'b110 : res = a | immI;                                  // ORI
                        3'b111 : res = a & immI;                                  // ANDI
                        3'b001 : res = a << sh;                                   // SLLI
                        // NOTE: these must be separate statements. In a ternary
                        // an unsigned arm makes the whole expression unsigned,
                        // which silently demotes >>> to a logical shift.
                        3'b101 : if(f7 == 7'b0100000) res = $signed(a) >>> sh;    // SRAI
                                 else                 res = a >> sh;              // SRLI
                        default: res = 32'd0;
                    endcase
                    wen = 1'b1;
                end
                //----------------------------------------------------- OP ----
                7'b0110011 : begin
                    if(f7 == 7'b0000001) begin
                        case(f3)                        // ---- RV32M ----
                            3'b000 : res = f_mul   (a,b);   // MUL
                            3'b001 : res = f_mulh  (a,b);   // MULH
                            3'b010 : res = f_mulhsu(a,b);   // MULHSU
                            3'b011 : res = f_mulhu (a,b);   // MULHU
                            3'b100 : res = f_div   (a,b);   // DIV
                            3'b101 : res = f_divu  (a,b);   // DIVU
                            3'b110 : res = f_rem   (a,b);   // REM
                            3'b111 : res = f_remu  (a,b);   // REMU
                        endcase
                    end
                    else begin
                        case(f3)                        // ---- RV32I ----
                            3'b000 : res = (f7 == 7'b0100000) ? (a - b) : (a + b);
                            3'b001 : res = a << b[4:0];
                            3'b010 : res = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
                            3'b011 : res = (a < b) ? 32'd1 : 32'd0;
                            3'b100 : res = a ^ b;
                            // separate statements, see the SRAI note above
                            3'b101 : if(f7 == 7'b0100000) res = $signed(a) >>> b[4:0];
                                     else                 res = a >> b[4:0];
                            3'b110 : res = a | b;
                            3'b111 : res = a & b;
                        endcase
                    end
                    wen = 1'b1;
                end
                //------------------------------------------------ unknown ----
                default : begin
                    halted = 1'b1;
                    nextpc = pc;   // the DUT spins on an unrecognised opcode
                end
            endcase

            // x0 is hardwired to zero: writes to it are discarded.
            if(wen && (rd != 5'd0)) regs[rd] = res;
            pc = nextpc;
        end
    endtask

endmodule
