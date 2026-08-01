//=============================================================================
// Multi-cycle 32-bit RISC-V processor : RV32I base + RV32M (mul/div)
//
// Execution model: one instruction takes several clock cycles, sequenced by
// the FSM in the `controller` module:
//
//   fetch -> decode -> read_src_regs -> execute -> [rw_data_mem] -> write_back
//         -> call_nxt_pc -> fetch ...
//
// `pc` and `instr` stay constant for the whole of one instruction, so every
// combinational signal derived from `instr` (WBMUX, IRMUX, immediates, ...)
// is stable across all the states of that instruction. The write-back and
// next-PC logic both rely on that property.
//=============================================================================

// RISC-V processor (Top module)
module risc_v(
    output [31:0] pc,instr,ALUOUT,
    output [4:0]  RS1_addr,RS2_addr,RD_addr,
    output [31:0] Jump_target,
    output        Branch_target,MWR,WERF,
    output [31:0] data_mem_out,write_data,
    output [2:0]  WBMUX,
    output [4:0]  ALUOP,   // widened 4->5 bits: RV32M adds 8 more ALU operations
    output [1:0]  IRMUX,PCMUX,
    input         clock,reset,
    // Instruction memory load port: hold the core in reset and stream in one
    // instruction per clock with imem_load_en high (see instruction_mem).
    input         imem_load_en,
    input  [31:0] imem_load_data);

    reg  [31:0] WD;                                  // write-back data
    wire [31:0] immI,immB,immJ,immS,immU,rs1,rs2;
    wire [11:0] imm_bus;
    wire [19:0] imm_bus_UJ;
    wire [4:0]  dec_aluop;
    wire        BJ_sel;

    // Sub-word load/store support (LB/LH/LW/LBU/LHU and SB/SH/SW).
    wire [31:0] store_data;   // rs2 replicated into the addressed byte lane(s)
    wire [3:0]  store_strb;   // per-byte write strobes for the addressed word
    wire [31:0] load_result;  // memory word narrowed to the access size, then extended
    // MWR is low only during rw_data_mem of a Store, so it qualifies the strobes.
    wire [3:0]  dmem_wstrb = MWR ? 4'b0000 : store_strb;

    /* Write-back source mux.
       The old version of this block was a 2-stage shift/OR "hold" network that
       stretched the write data over two clock pulses, because the value it had
       to capture (ALU result / loaded word) was only valid for a single state.
       That is no longer needed: ALUOP is now held for the whole instruction
       (see controller) and the data memory read is combinational, so every
       source below is stable from `execute` onward - i.e. still valid during
       `write_back`, which is the only state where WERF is asserted. A plain
       combinational mux is therefore correct and much easier to reason about. */
    always@(*) begin
        case(WBMUX)
            3'b000 : WD = pc + 32'd4;   // JAL / JALR : return address
            3'b001 : WD = immU;         // LUI
            3'b010 : WD = pc + immU;    // AUIPC
            3'b011 : WD = ALUOUT;       // R-type / I-type
            3'b100 : WD = load_result;  // Load (already size-narrowed and extended)
            default: WD = ALUOUT;
        endcase
    end

    assign write_data = WD;  // for observability purpose

    // instantiating other modules
    prgm_cntr program_counter(pc, immB, immJ, Jump_target[31:1], PCMUX, BJ_sel, clock, reset);
    instruction_mem instruction_memory(instr, pc, clock, reset, imem_load_en, imem_load_data);
    instruction_dec instruction_decoder(dec_aluop, imm_bus, imm_bus_UJ, RS1_addr, RS2_addr, RD_addr, instr, clock);
    reg_file register_file(rs1, rs2, reset, clock, WERF, WD, RD_addr, RS1_addr, RS2_addr);
    ALU alu(ALUOUT, Jump_target, Branch_target, rs1, rs2, immS, immI, instr[6:0], IRMUX, ALUOP);
    controller FSM(ALUOP, WERF, MWR, BJ_sel, IRMUX, PCMUX, WBMUX, dec_aluop, instr[6:0], instr[12], clock, reset, Branch_target);
    data_mem data_memory(data_mem_out, reset, clock, dmem_wstrb, ALUOUT, store_data);
    imm_gen immediate_generator(immB, immJ, immI, immS, immU, imm_bus, imm_bus_UJ, instr[6:0]);
    // data_mem_out is the raw 32-bit word at the addressed location; lsu_align
    // turns it into the value a Load actually writes back.
    lsu_align load_store_align(store_data, store_strb, load_result, data_mem_out, rs2, ALUOUT[1:0], instr[14:12]);
endmodule


//=============================================================================
// ALU
//
// op_code encoding (5 bits, driven by the instruction decoder):
//   0  ADD        6  AND       9  SLL      12 SLT (signed)
//   1  SUB        7  OR       10  SRL      13 SLTU
//                 8  XOR      11  SRA      14 EQ  (branch compare)
//
//   RV32M ops are 5'b10_<funct3>, so the decoder can pass funct3 straight
//   through:
//   16 MUL   17 MULH   18 MULHSU   19 MULHU
//   20 DIV   21 DIVU   22 REM      23 REMU
//=============================================================================
module ALU(
    output reg [31:0] ALU_out,
    output reg [31:0] JT,          // Jump Target
    output            BT,          // Branch compare result (raw, un-inverted)
    input      [31:0] A,rs2,       // RS1,RS2
    input      [31:0] immS,immI,
    input      [6:0]  instr,       // opcode field
    input      [1:0]  IRMUX,       // taken from controller
    input      [4:0]  op_code);    // From instruction decoder

    reg  [31:0] B;
    wire [31:0] add_sub,mult_HB,mult_LB,div_R,div_Q;
    wire signed [31:0] A_Sign = A;
    wire signed [31:0] B_Sign = B;
    reg  c_in;

    // RV32I only ever shifts by rs2[4:0] / shamt[4:0]. Masking here fixes both
    // the register shifts (SLL/SRL/SRA with rs2 >= 32) and SRAI, whose shamt
    // field shares instr[31:20] with funct7=0100000 and so arrives polluted.
    wire [4:0] shamt = B[4:0];

    // Multiply/divide unit enables and operand signedness, decoded from op_code.
    // op_code[4] selects the M-extension block, op_code[2] picks divide vs multiply.
    wire mult_en    = op_code[4] & ~op_code[2];           // 16..19
    wire div_en     = op_code[4] &  op_code[2];           // 20..23
    wire a_signed   = ~(op_code[1] & op_code[0]);         // MULHU  treats rs1 as unsigned
    wire b_signed   = ~op_code[1];                        // MULHSU/MULHU treat rs2 as unsigned
    wire div_signed = ~op_code[0];                        // DIV/REM signed, DIVU/REMU unsigned

    assign BT = ALU_out[0];

    carry_look_ahead ADD(add_sub, A, B, c_in);
    Booths_mult      MULT({mult_HB,mult_LB}, mult_en, a_signed, b_signed, A, B);
    non_rest_div     DIV(div_R, div_Q, A, B, div_signed, div_en);

    // logic to select among immediate values and rs2
    always@(*) begin
        case(IRMUX)
            2'b00 : B = immS;
            2'b01 : B = immI;
            2'b10 : B = rs2;
            default: B = rs2;
        endcase
    end

    always@(*) begin
        ALU_out = 32'd0; // default assignment
        c_in    = 1'b0;  // to avoid latch
        JT      = 32'd0;

        case(op_code)
            5'd0 : begin
                c_in    = 1'b0;
                ALU_out = add_sub; // Addition
                JT = (instr == 7'b1100111) ? add_sub : 32'd0; // Jump Target for JALR instruction
            end
            5'd1 : begin
                c_in    = 1'b1;
                ALU_out = add_sub; // Subtraction (A + ~B + 1)
            end
            5'd6 : ALU_out = A & B;                // AND
            5'd7 : ALU_out = A | B;                // OR
            5'd8 : ALU_out = A ^ B;                // XOR
            5'd9 : ALU_out = A << shamt;           // Left Shift
            5'd10: ALU_out = A >> shamt;           // Right Shift Logical
            5'd11: ALU_out = A_Sign >>> shamt;     // Right Shift Arithmetic (needs signed LHS)
            5'd12: ALU_out[0] = (A_Sign < B_Sign); // Set Less Than Signed
            5'd13: ALU_out[0] = (A < B);           // Set Less Than Unsigned
            5'd14: ALU_out[0] = (A == B);          // Equal (branch compare)

            // ---- RV32M ----
            5'd16       : ALU_out = mult_LB;  // MUL : low 32 bits
            5'd17,5'd18,
            5'd19       : ALU_out = mult_HB;  // MULH / MULHSU / MULHU : high 32 bits
            5'd20,5'd21 : ALU_out = div_Q;    // DIV / DIVU
            5'd22,5'd23 : ALU_out = div_R;    // REM / REMU
            default :;
        endcase
    end
endmodule


//=============================================================================
// Controller (FSM)
//=============================================================================
module controller(
    output [4:0]     ALUOP,
    output reg       WERF,MWR,BJ_sel,
    output reg [1:0] IRMUX,PCMUX,
    output reg [2:0] WBMUX,
    input      [4:0] dec_aluop,
    input      [6:0] instr,
    input            b_inv,   // funct3[0] of a branch: inverts the compare result
    input            fsm_clk,fsm_rst,BT);

    parameter fetch = 3'b000,
    decode = 3'b001,
    read_src_regs = 3'b010,
    execute = 3'b011,
    rw_data_mem = 3'b100,
    write_back = 3'b101,
    call_nxt_pc = 3'b110;

    reg [2:0] present_state,next_state;

    // sequential logic for Present state
    always@(posedge fsm_clk) begin
        if(fsm_rst) present_state <= fetch;
        else present_state <= next_state;
    end

    // combinational logic for next state
    always@(*) begin
        case(present_state)
            3'b000 : next_state = decode;
            3'b001 : begin
                if((instr==7'b0110011) || (instr==7'b0010011) ||
                (instr==7'b0000011) || (instr==7'b0100011) ||
                (instr==7'b1100011) || (instr==7'b1100111)) next_state = read_src_regs;
                else if(instr==7'b0010111) next_state = execute; // AUIPC instruction
                else if((instr==7'b0110111) || (instr==7'b1101111)) next_state = write_back; // LUI and JAL instruction
                else next_state = fetch; // unrecognised opcode -> spins here (acts as a halt)
            end
            3'b010 : next_state = execute;
            3'b011 : begin
                if(instr==7'b0000011 || instr==7'b0100011) next_state = rw_data_mem; // Load and Store instructions
                else if(instr==7'b1100011) next_state = call_nxt_pc; // Branch instructions
                else if((instr==7'b1100111) ||(instr==7'b0110011) ||
                (instr==7'b0010011) ||(instr==7'b0110111) || (instr==7'b0010111)) next_state = write_back; // JALR,R,I,LUI and AUIPC instruction
                else next_state = fetch; // default assignment
            end
            3'b100 : begin
                if(instr==7'b0100011) next_state = call_nxt_pc;
                else next_state = write_back;
            end
            3'b101 : next_state = call_nxt_pc;
            3'b110 : next_state = fetch;
            default: next_state = fetch;
        endcase
    end

    // logic for output signals
    // ALUOP is held for the whole instruction rather than only during `execute`.
    // The ALU is purely combinational, and two consumers need its result after
    // execute has passed: the data memory address (during rw_data_mem) and the
    // branch compare BT (during call_nxt_pc). Gating ALUOP off outside execute
    // forced ALU_out to 0 in both of those states.
    assign ALUOP = dec_aluop;

    // logic for IRMUX
    always@(*) begin
        case(instr)
            7'b0110011 : IRMUX = 2'b10; // takes rs2 value in R-type instruction
            7'b0010011 : IRMUX = 2'b01; // takes immediate value in I-type instruction
            7'b0000011 : IRMUX = 2'b01; // takes imm value to calculate data mem address in Load instruction
            7'b0100011 : IRMUX = 2'b00; // takes imm value to calculate data mem address in Store instruction
            7'b1100111 : IRMUX = 2'b01; // takes imm value in JALR instruction
            default: IRMUX = 2'b10;
        endcase
    end

    // logic for MWR --> ~MWR=Write MWR=Read
    // Default is 1 (read) rather than z: the data memory qualifies its write
    // with !MWR, and an x/z there would make the write condition unknown.
    always@(*) begin
        if(present_state==rw_data_mem) begin
            if(instr==7'b0000011) MWR = 1'b1;      // Read operation (Load)
            else if(instr==7'b0100011) MWR = 1'b0; // Write operation (Store)
            else MWR = 1'b1;
        end
        else MWR = 1'b1;
    end

    // logic for WBMUX signal
    always@(*) begin
        case(instr)
            7'b0110011 : WBMUX = 3'b011; // write back alu o/p into reg file (R)
            7'b0010011 : WBMUX = 3'b011; // write back alu o/p into reg file (I)
            7'b0000011 : WBMUX = 3'b100; // write back data mem o/p into reg file (Load)
            7'b0100011 : WBMUX = 3'b011; // no write back (Store)  - WERF stays 0, value is a don't care
            7'b1100011 : WBMUX = 3'b011; // no write back (Branch) - WERF stays 0, value is a don't care
            7'b1101111 : WBMUX = 3'b000; // Write back pc+4 value into reg file (JAL)
            7'b1100111 : WBMUX = 3'b000; // Write back pc+4 value into reg file (JALR)
            7'b0110111 : WBMUX = 3'b001; // Write back IMMU into reg file (LUI)
            7'b0010111 : WBMUX = 3'b010; // Write back pc+IMMU into reg file (AUIPC)
            default: WBMUX = 3'b011;
        endcase
    end

    // logic for WERF signal
    always@(*) begin
        if(present_state==write_back) begin
            case(instr)
                7'b0110011 : WERF = 1'b1; // write back alu o/p into reg file (R)
                7'b0010011 : WERF = 1'b1; // write back alu o/p into reg file (I)
                7'b0000011 : WERF = 1'b1; // write back data mem o/p into reg file (Load)
                7'b0100011 : WERF = 1'b0; // no need to write back into reg file (Store)
                7'b1100011 : WERF = 1'b0; // no need to write back into reg file (Branch)
                7'b1101111 : WERF = 1'b1; // Write back pc+4 value into reg file (JAL)
                7'b1100111 : WERF = 1'b1; // Write back pc+4 value into reg file (JALR)
                7'b0110111 : WERF = 1'b1; // Write back IMMU into reg file (LUI)
                7'b0010111 : WERF = 1'b1; // Write back pc+IMMU into reg file (AUIPC)
                default: WERF = 1'b0;
            endcase
        end
        else WERF = 1'b0;
    end

    // logic for PCMUX
    always@(*) begin
        if(present_state==call_nxt_pc) begin
            case(instr)
                7'b1100011 : begin // Branch instruction
                    // RV32I pairs the six branches so that each pair shares one
                    // comparator and differs only in funct3[0]:
                    //   BEQ/BNE -> EQ, BLT/BGE -> SLT, BLTU/BGEU -> SLTU.
                    // b_inv (= funct3[0]) inverts the compare for BNE/BGE/BGEU.
                    if(BT ^ b_inv) PCMUX = 2'b10; // taken     : pc = pc+IMMB
                    else PCMUX = 2'b01;           // not taken : pc = pc+4
                    BJ_sel = 1'b0; // selects sxt(immB) in Branch instruction
                end
                7'b1101111 : begin // JAL instruction
                    PCMUX  = 2'b10; // pc=pc+IMMJ
                    BJ_sel = 1'b1;  // selects sxt(immJ) in JAL instruction
                end
                7'b1100111 : begin // JALR instruction
                    PCMUX  = 2'b11; // JT=rs1+IMMI, pc={JT[31:1],1'b0}
                    BJ_sel = 1'b0;  // explicit assignment to avoid latch
                end
                // R,I,Load,Store,LUI,AUIPC all just fall through to pc+4.
                7'b0110011, 7'b0010011, 7'b0000011,
                7'b0100011, 7'b0110111, 7'b0010111: begin
                    PCMUX  = 2'b01; // pc=pc+4
                    BJ_sel = 1'b0;  // explicit assignment to avoid latch
                end
                default: begin
                    PCMUX  = 2'b00; // pc=pc (unrecognised opcode -> halt)
                    BJ_sel = 1'b0;  // default assignment
                end
            endcase
        end
        else begin
            // Every other state holds the PC. In particular JAL/JALR must NOT
            // bump the PC during write_back: pc still has to hold the address of
            // the jump itself so that write-back can form pc+4 and call_nxt_pc
            // can form pc+immJ from the correct base.
            PCMUX  = 2'b00; // pc=pc
            BJ_sel = 1'b0;  // default assignment
        end
    end
endmodule


//=============================================================================
// Instruction decoder
//=============================================================================
module instruction_dec(
    output reg [4:0]  ALUOP,
    output reg [11:0] imm_bus,
    output reg [19:0] imm_bus_UJ,
    output reg [4:0]  source_addr1,source_addr2,destination_addr,
    input      [31:0] instr,
    input             dec_clk);

    // Logic to decode ALUOP signal
    always@(*) begin
        case(instr[6:0]) // opcode
            7'b0110011 : begin // R-type instr
                if(instr[31:25]==7'b0000001) begin
                    // RV32M: funct7=0000001, funct3 selects the operation.
                    // ALUOP = {2'b10, funct3} maps straight onto the ALU codes
                    // 16..23 (MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU).
                    ALUOP = {2'b10, instr[14:12]};
                end
                else begin
                    case(instr[14:12]) // func3
                        3'b000 : begin
                            // instr[31:25] = func7
                            if(instr[31:25]==7'b0100000) ALUOP = 5'd1; // SUB
                            else ALUOP = 5'd0;                         // ADD
                        end
                        3'b001 : ALUOP = 5'd9;  // Shift Left Logical
                        3'b010 : ALUOP = 5'd12; // Set Less Than
                        3'b011 : ALUOP = 5'd13; // Set Less Than (U)
                        3'b100 : ALUOP = 5'd8;  // XOR
                        3'b101 : begin
                            if(instr[31:25]==7'b0100000) ALUOP = 5'd11; // Shift Right Arith
                            else ALUOP = 5'd10;                         // Shift Right Logical
                        end
                        3'b110 : ALUOP = 5'd7;  // OR
                        3'b111 : ALUOP = 5'd6;  // AND
                    endcase
                end
            end
            7'b0010011 : begin // I-type instr
                case(instr[14:12]) // func3
                    3'b000 : ALUOP = 5'd0;  // ADD_Imm
                    3'b001 : ALUOP = 5'd9;  // Shift Left Logical Imm
                    3'b010 : ALUOP = 5'd12; // Set Less Than Imm
                    3'b011 : ALUOP = 5'd13; // Set Less Than Imm (U)
                    3'b100 : ALUOP = 5'd8;  // XOR Imm
                    3'b101 : begin
                        if(instr[31:25]==7'b0100000) ALUOP = 5'd11; // Shift Right Arith Imm
                        else ALUOP = 5'd10;                         // Shift Right Logical Imm
                    end
                    3'b110 : ALUOP = 5'd7;  // OR Imm
                    3'b111 : ALUOP = 5'd6;  // AND Imm
                endcase
            end
            7'b0000011 : ALUOP = 5'd0; // I-type Load instr (addr = rs1+immI)
            7'b0100011 : ALUOP = 5'd0; // Store instr       (addr = rs1+immS)
            7'b1100011 : begin // B-Type instr
                // Only the comparator is chosen here; the funct3[0] bit that
                // inverts the result is handled in the controller.
                //   000 BEQ  / 001 BNE  -> EQ
                //   100 BLT  / 101 BGE  -> SLT  (signed)
                //   110 BLTU / 111 BGEU -> SLTU (unsigned)
                case(instr[14:12])
                    3'b000, 3'b001 : ALUOP = 5'd14; // Equal
                    3'b100, 3'b101 : ALUOP = 5'd12; // Less Than (S)
                    3'b110, 3'b111 : ALUOP = 5'd13; // Less Than (U)
                    default        : ALUOP = 5'd14;
                endcase
            end
            7'b1100111 : ALUOP = 5'd0; // JT=rs1+immI (JALR)
            default    : ALUOP = 5'd0;
        endcase
    end

    // Logic to decode immediate bus and address of source and destination registers.
    // Unused register fields are driven to x0 (5'd0) rather than z, so an unused
    // port can never index the register file with an unknown address.
    always@(posedge dec_clk) begin
        case(instr[6:0]) // opcode
            7'b0110011 : begin // R-type instr
                destination_addr <= instr[11:7];
                source_addr1     <= instr[19:15];
                source_addr2     <= instr[24:20];
                imm_bus          <= 12'd0;
                imm_bus_UJ       <= 20'd0;
            end
            7'b0010011 : begin // I-type instr
                destination_addr <= instr[11:7];
                source_addr1     <= instr[19:15];
                source_addr2     <= 5'd0;
                imm_bus          <= instr[31:20];
                imm_bus_UJ       <= 20'd0;
            end
            7'b0000011 : begin // Load instr
                destination_addr <= instr[11:7];
                source_addr1     <= instr[19:15];
                source_addr2     <= 5'd0;
                imm_bus          <= instr[31:20];
                imm_bus_UJ       <= 20'd0;
            end
            7'b0100011 : begin // Store instr
                destination_addr <= 5'd0;
                source_addr1     <= instr[19:15];
                source_addr2     <= instr[24:20];
                imm_bus          <= {instr[31:25],instr[11:7]};
                imm_bus_UJ       <= 20'd0;
            end
            7'b1100011 : begin // B-type instr
                imm_bus          <= {instr[31],instr[7],instr[30:25],instr[11:8]};
                destination_addr <= 5'd0;
                source_addr1     <= instr[19:15];
                source_addr2     <= instr[24:20];
                imm_bus_UJ       <= 20'd0;
            end
            7'b1101111 : begin // Jump and Link (JAL)
                {source_addr1,source_addr2} <= 10'd0;
                imm_bus          <= 12'd0;
                imm_bus_UJ       <= {instr[31],instr[19:12],instr[20],instr[30:21]};
                destination_addr <= instr[11:7];
            end
            7'b1100111 : begin // Jump and Link Reg (JALR)
                // JALR is an I-type instruction: it reads rs1 and uses immI.
                // (It does not use the J-type immediate at all.)
                source_addr1     <= instr[19:15];
                source_addr2     <= 5'd0;
                imm_bus          <= instr[31:20];
                imm_bus_UJ       <= 20'd0;
                destination_addr <= instr[11:7];
            end
            7'b0110111 : begin // LUI
                {source_addr1,source_addr2} <= 10'd0;
                imm_bus          <= 12'd0;
                imm_bus_UJ       <= instr[31:12];
                destination_addr <= instr[11:7];
            end
            7'b0010111 : begin // AUIPC
                {source_addr1,source_addr2} <= 10'd0;
                imm_bus          <= 12'd0;
                imm_bus_UJ       <= instr[31:12];
                destination_addr <= instr[11:7];
            end
            default : begin
                destination_addr <= 5'd0;
                source_addr1     <= 5'd0;
                source_addr2     <= 5'd0;
                imm_bus          <= 12'd0;
                imm_bus_UJ       <= 20'd0;
            end
        endcase
    end
endmodule


//=============================================================================
// Data memory : 64 words (256 bytes), word addressed with byte write enables
//   - write is synchronous (posedge) and per-byte, so SB/SH can update part of
//     a word without disturbing the neighbouring bytes
//   - read is combinational and always returns the whole word; narrowing it to
//     the access size is lsu_align's job. A registered read output would arrive
//     one cycle after write_back has already latched the register file.
//=============================================================================
module data_mem(
    output [31:0] data_out,
    input         data_mem_rst,data_mem_clk,
    input  [3:0]  wstrb,      // per-byte write strobes; 4'b0000 --> read only
    input  [31:0] addr,       // byte address (from the ALU)
    input  [31:0] data_in);

    parameter DEPTH = 64;

    reg [31:0] mem[DEPTH-1:0];
    integer i;

    // addr is a byte address; drop the two low bits to index words.
    wire [5:0] word_addr = addr[7:2];

    assign data_out = mem[word_addr];

    always@(posedge data_mem_clk) begin
        if(data_mem_rst) begin
            for(i=0;i<DEPTH;i=i+1) mem[i] <= 32'd0;
        end
        else begin
            if(wstrb[0]) mem[word_addr][7:0]   <= data_in[7:0];
            if(wstrb[1]) mem[word_addr][15:8]  <= data_in[15:8];
            if(wstrb[2]) mem[word_addr][23:16] <= data_in[23:16];
            if(wstrb[3]) mem[word_addr][31:24] <= data_in[31:24];
        end
    end
endmodule


//=============================================================================
// Load / Store alignment unit
//
// The data memory is organised as 32-bit words, but RV32I addresses memory by
// byte. This block sits between the two and handles both directions.
//
// funct3 encoding (RV32I, unprivileged spec):
//     Loads : 000 LB   001 LH   010 LW   100 LBU  101 LHU
//     Stores: 000 SB   001 SH   010 SW
//   so funct3[1:0] is the access size (00 byte, 01 half, 10 word) and
//   funct3[2] selects zero-extension instead of sign-extension on a Load.
//
// Store side: rs2 is replicated across all four byte lanes and the strobes
// pick out the lane(s) the address actually selects, so no shifter is needed.
//
// NOTE on misaligned accesses: this core has no trap/CSR mechanism, so it
// cannot raise the address-misaligned exception the spec permits. Behaviour is
// therefore defined only for naturally-aligned accesses (LH/SH on even
// addresses, LW/SW on multiples of 4). A misaligned access will neither trap
// nor produce a spanning result - it silently accesses the containing word.
//=============================================================================
module lsu_align(
    output reg [31:0] store_data,   // rs2 replicated into the addressed lane(s)
    output reg [3:0]  store_strb,   // per-byte write strobes
    output reg [31:0] load_result,  // sub-word read, sign/zero extended
    input      [31:0] mem_word,     // raw 32-bit word read from the data memory
    input      [31:0] rs2,          // store source
    input      [1:0]  byte_off,     // addr[1:0]
    input      [2:0]  funct3);

    reg [7:0]  ld_byte;
    reg [15:0] ld_half;

    always@(*) begin
        // ---- store path ----
        case(funct3[1:0])
            2'b00 : begin // SB
                store_data = {4{rs2[7:0]}};
                store_strb = 4'b0001 << byte_off;
            end
            2'b01 : begin // SH
                store_data = {2{rs2[15:0]}};
                store_strb = 4'b0011 << byte_off;
            end
            default : begin // SW
                store_data = rs2;
                store_strb = 4'b1111;
            end
        endcase

        // ---- load path : select the addressed sub-word ... ----
        case(byte_off)
            2'b00 : ld_byte = mem_word[7:0];
            2'b01 : ld_byte = mem_word[15:8];
            2'b10 : ld_byte = mem_word[23:16];
            2'b11 : ld_byte = mem_word[31:24];
        endcase
        ld_half = byte_off[1] ? mem_word[31:16] : mem_word[15:0];

        // ---- ... then extend it to 32 bits ----
        case(funct3)
            3'b000 : load_result = {{24{ld_byte[7]}},  ld_byte}; // LB  : sign extend
            3'b001 : load_result = {{16{ld_half[15]}}, ld_half}; // LH  : sign extend
            3'b100 : load_result = {24'd0, ld_byte};             // LBU : zero extend
            3'b101 : load_result = {16'd0, ld_half};             // LHU : zero extend
            default: load_result = mem_word;                     // LW  (funct3 = 010)
        endcase
    end
endmodule


//=============================================================================
// Instruction memory : 256 words (1 KB of instruction space)
//
// Load port: while `load_en` is high, one instruction word is written per
// clock at an internally incrementing pointer, so a testbench can simply
// stream a program in without having to manage addresses:
//
//     rst = 1;                          // clears the write pointer
//     @(negedge clk);
//     load_en = 1;
//     load_data = <instr 0>; @(negedge clk);
//     load_data = <instr 1>; @(negedge clk);
//     ...
//     load_en = 0; rst = 0;             // release the core, it starts at pc=0
//
// rst clears only the pointer, not the array, so the core can be reset and
// re-run without reloading the program.
//
// The read port is combinational: the FSM expects `instr` to be valid during
// `fetch`, and every decode-time control signal derives from it.
//=============================================================================
module instruction_mem(
    output [31:0] instruction,
    input  [31:0] addr,        // byte address (from the PC)
    input         clk,
    input         rst,         // resets the load pointer
    input         load_en,
    input  [31:0] load_data);

    parameter DEPTH = 256;     // instruction words
    parameter PTRW  = 8;       // log2(DEPTH)

    reg [31:0]     mem[DEPTH-1:0];
    reg [PTRW-1:0] wr_ptr;

    assign instruction = mem[addr[PTRW+1:2]]; // byte address -> word index

    always@(posedge clk) begin
        if(load_en) begin
            mem[wr_ptr] <= load_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end
        else if(rst) wr_ptr <= {PTRW{1'b0}};
    end
endmodule


//=============================================================================
// Register file : 32 x 32-bit, x0 hardwired to zero
//=============================================================================
module reg_file(
    output reg [31:0] rs1,rs2,
    input             reg_file_rst,reg_file_clk,write_enb,
    input      [31:0] write_data,
    input      [4:0]  write_addr,read_addr1,read_addr2);

    reg [31:0] mem[31:0]; // memory cells
    integer i;

    // Synchronous write operation. x0 stays zero simply by never being written,
    // which is synthesisable (the old always@(mem[0]) trick was not).
    always@(posedge reg_file_clk) begin
        if(reg_file_rst) begin
            for(i=0;i<32;i=i+1) mem[i] <= 32'd0;
        end
        else if(write_enb && (write_addr != 5'd0)) mem[write_addr] <= write_data;
    end

    // Asynchronous read operation
    always@(*) begin
        rs1 = mem[read_addr1];
        rs2 = mem[read_addr2];
    end
endmodule


//=============================================================================
// Program counter
//=============================================================================
module prgm_cntr(
    output reg [31:0] pc,
    input      [31:0] immB,immJ,
    input      [30:0] JT,
    input      [1:0]  PCMUX,
    input             BJ_sel,pc_clk,pc_rst);

    always@(posedge pc_clk) begin
        if(pc_rst) pc <= 32'd0;
        else begin
            case(PCMUX)
                2'b00 : pc <= pc;
                2'b01 : pc <= pc + 32'd4;
                2'b10 : begin
                    if(!BJ_sel) pc <= pc + immB; // Branch taken
                    else pc <= pc + immJ;        // JAL
                end
                2'b11 : pc <= {JT,1'b0};         // JALR : low bit is cleared
            endcase
        end
    end
endmodule


//=============================================================================
// Immediate value generating block
//=============================================================================
module imm_gen(
    output reg [31:0] immB,immJ,immI,immS,immU,
    input      [11:0] imm1,
    input      [19:0] imm2,
    input      [6:0]  opcode);

    always@(*) begin
        immB = 32'd0;
        immJ = 32'd0;
        immI = 32'd0;
        immS = 32'd0;
        immU = 32'd0;
        case(opcode)
            7'b0010011, 7'b0000011, 7'b1100111 : immI = {{20{imm1[11]}},imm1};  // I, Load, JALR-type instruction
            7'b0100011 : immS = {{20{imm1[11]}},imm1};                          // Store instruction
            7'b1100011 : immB = {{19{imm1[11]}},imm1,1'b0};                     // Branch instruction
            7'b1101111 : immJ = {{11{imm2[19]}},imm2,1'b0};                     // JAL instruction
            7'b0110111, 7'b0010111 : immU = {imm2,12'd0};                       // LUI / AUIPC
            default : ; // to avoid violation
        endcase
    end
endmodule


//=============================================================================
// Carry Look Ahead Adder / Subtractor
//   cin = 0 -> result = a + b
//   cin = 1 -> result = a + ~b + 1 = a - b
//=============================================================================
module carry_look_ahead(
    output reg [31:0] result,
    input      [31:0] a,b,
    input             cin);

    reg [31:0] p,g,r,y,c_o,B;

    integer i,j,k;
    always@(*) begin
        for(k=0;k<32;k=k+1) begin
            // For subtraction cin inverts b; the +1 enters as the carry-in.
            // NOTE: the operands are used exactly as given. The previous version
            // swapped them by magnitude when subtracting, which computed |a-b|
            // instead of a-b whenever a < b.
            B[k] = b[k]^cin;
            // carry generate and propagate.
            p[k] = a[k]^B[k];
            g[k] = a[k]&B[k];
        end
        r = {g[30:0],cin};
        // Logic for carry generator.
        for(k=0;k<=31;k=k+1) begin
            c_o[k] = g[k];
            for(i=0;i<=k;i=i+1) begin
                y[i] = r[i];
                for(j=i;j<=k;j=j+1) begin
                    y[i] = y[i]&p[j];
                end
                c_o[k] = c_o[k]|y[i];
            end
        end
        // iteration for sum signal.
        result[0] = p[0]^cin;
        for(k=1;k<32;k=k+1) result[k] = p[k]^c_o[k-1];
    end
endmodule


//=============================================================================
// Booth's Algorithm Multiplier (radix-2), 32x32 -> 64
//
// Expressed as Booth recoding over the multiplier bits, accumulating shifted
// partial products. Each adjacent bit pair {q[i+1],q[i]} selects +M, -M or 0:
//     00 / 11 -> 0      01 -> +M      10 -> -M
//
// To cover all four RV32M forms, both operands are first widened to 33 bits,
// sign-extended or zero-extended according to a_signed/b_signed:
//     MUL/MULH : signed x signed        MULHSU : signed x unsigned
//     MULHU    : unsigned x unsigned
// The 33x33 product of the widened values is exact in 64 bits in every case.
//=============================================================================
module Booths_mult(
    output reg [63:0] mult,
    input             enb,
    input             a_signed,b_signed,
    input      [31:0] M,Q);   // M-->Multiplicand Q-->Multiplier

    reg [32:0] m_ext;         // multiplicand widened to 33 bits
    reg [33:0] q_ext;         // multiplier widened to 33 bits, plus Q(-1)
    reg [63:0] pp;            // m_ext sign-extended to 64 bits
    reg [63:0] acc;
    integer i;

    always@(*) begin
        // default assignments on every path, so no latches are inferred
        m_ext = 33'd0;
        q_ext = 34'd0;
        pp    = 64'd0;
        acc   = 64'd0;

        if(!enb) mult = 64'd0;
        else begin
            m_ext = {a_signed & M[31], M};
            q_ext = {b_signed & Q[31], Q, 1'b0}; // low bit is the implicit Q(-1)=0
            pp    = {{31{m_ext[32]}}, m_ext};

            acc = 64'd0;
            for(i=0;i<33;i=i+1) begin
                case({q_ext[i+1],q_ext[i]})
                    2'b01 : acc = acc + (pp << i); // end of a run of 1s   : +M
                    2'b10 : acc = acc - (pp << i); // start of a run of 1s : -M
                    default: acc = acc;            // 00 / 11 : no operation
                endcase
            end
            mult = acc;
        end
    end
endmodule


//=============================================================================
// Non-Restoring Division, 32/32 -> 32-bit quotient + 32-bit remainder
//
// The core loop divides unsigned magnitudes; the signs are applied afterwards:
//     quotient sign  = sign(a) ^ sign(b)
//     remainder sign = sign(a)      (RV32M: the remainder follows the dividend)
//
// RV32M special cases:
//     b == 0     -> quotient = all ones, remainder = a  (no trap)
//     -2^31 / -1 -> quotient = -2^31, remainder = 0. This falls out of the
//                   magnitude path on its own: |-2^31| is 32'h80000000, the
//                   division by 1 gives 32'h80000000, and the sign is positive
//                   (neg ^ neg), so no special case is needed.
//=============================================================================
module non_rest_div(
    output reg [31:0] R,Q,
    input      [31:0] a,b,      // a--> Dividend b--> Divisor
    input             is_signed,
    input             rst);

    reg [33:0] acc;             // partial remainder (wide enough for the shift)
    reg [31:0] dvd_mag,dvr_mag,q;
    reg a_neg,b_neg;
    integer n;

    always@(*) begin
        // default assignments on every path, so no latches are inferred
        a_neg   = 1'b0;
        b_neg   = 1'b0;
        dvd_mag = 32'd0;
        dvr_mag = 32'd0;
        acc     = 34'd0;
        q       = 32'd0;

        if(!rst) begin
            R = 32'd0;
            Q = 32'd0;
        end
        else if(b == 32'd0) begin
            // Division by zero, per the RISC-V spec (no trap is taken).
            Q = 32'hFFFFFFFF;
            R = a;
        end
        else begin
            a_neg   = is_signed & a[31];
            b_neg   = is_signed & b[31];
            // NOTE: the operands keep their roles. The previous version swapped
            // them by magnitude, so 3/10 computed 10/3.
            dvd_mag = a_neg ? (~a + 1'b1) : a;
            dvr_mag = b_neg ? (~b + 1'b1) : b;

            // ---- non-restoring loop on the magnitudes ----
            acc = 34'd0;
            q   = dvd_mag;
            for(n=0;n<32;n=n+1) begin
                {acc,q} = {acc,q} << 1;
                // Add or subtract the divisor depending on the sign of acc and
                // record the quotient bit; there is no restore step inside the
                // loop, hence "non-restoring".
                if(acc[33]) acc = acc + {2'b0,dvr_mag};
                else        acc = acc - {2'b0,dvr_mag};
                q[0] = ~acc[33];
            end
            // Final correction: if the remainder ended up negative, restore it once.
            if(acc[33]) acc = acc + {2'b0,dvr_mag};

            // ---- apply the signs ----
            Q = (a_neg ^ b_neg) ? (~q + 1'b1) : q;
            R = a_neg ? (~acc[31:0] + 1'b1) : acc[31:0];
        end
    end
endmodule
