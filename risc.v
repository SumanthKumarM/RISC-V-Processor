// RISC-V processor (Top module)
module risc_v(
    output [31:0]pc,instr,ALUOUT,
    output [4:0]RS1_addr,RS2_addr,RD_addr,
    output [31:0]Jump_target,
    output Branch_target,MWR,WERF,
    output [31:0]data_mem_out,write_data,
    output [2:0]WBMUX,
    output [3:0]ALUOP,
    output [1:0]IRMUX,PCMUX,
    input clock,reset);

    reg [31:0]temp,WD;
    wire [31:0]immI,immB,immJ,immS,immU,rs1,rs2;
    wire [11:0]imm_bus;
    wire [19:0]imm_bus_UJ;
    wire [3:0]dec_aluop;
    wire BJ_sel;

    /* logic to select write data and to extend the write data upto 2 clk pulses
       so that reg file recieves the data when data is obtained in execute state
       and write operation has to be done in write_back state */ 
    always@(posedge clock) begin
        temp<=ALUOUT;
        WD<=temp|ALUOUT;
        case (WBMUX)
            3'b000 : begin
                temp<=pc;
                WD<=temp|pc;
            end
            3'b001 : begin
                temp<=immU;
                WD<=temp|immU;
            end
            3'b010 : begin
                temp<=pc+immU;
                WD<=temp|(pc+immU);
            end
            3'b011 : begin
                temp<=ALUOUT;
                WD<=temp|ALUOUT;
            end
            3'b100 : begin
                temp<=data_mem_out;
                WD<=temp|data_mem_out;
            end
            default : ;
        endcase
    end
    
    assign write_data=WD;  // for observability purpose

    // instantiating other modules
    prgm_cntr program_counter(pc,immB,immJ,Jump_target[31:1],PCMUX,BJ_sel,clock,reset);
    instruction_mem instruction_memory(instr,pc);
    instruction_dec instruction_decoder(dec_aluop,imm_bus,imm_bus_UJ,RS1_addr,RS2_addr,RD_addr,instr,clock);
    reg_file register_file(rs1,rs2,reset,clock,WERF,WD,RD_addr,RS1_addr,RS2_addr);
    ALU alu(ALUOUT,Jump_target,Branch_target,rs1,rs2,immS,immI,instr[6:0],IRMUX,ALUOP);
    controller FSM(ALUOP,WERF,MWR,BJ_sel,IRMUX,PCMUX,WBMUX,dec_aluop,instr[6:0],clock,reset,Branch_target);
    data_mem data_memory(data_mem_out,reset,clock,MWR,ALUOUT,rs2);
    imm_gen immediate_generator(immB,immJ,immI,immS,immU,imm_bus,imm_bus_UJ,instr[6:0]);
endmodule

// ALU of the processor
module ALU(
    output reg [31:0]ALU_out,
    output reg [31:0]JT, // Jump Target
    output BT, // Branch Target
    input [31:0]A,rs2, // RS1,RS2
    input [31:0]immS,immI,
    input [6:0]instr,
    input [1:0]IRMUX, // taken from controller
    input [3:0]op_code); // From instruction decoder

    reg [31:0]B;
    wire [31:0]add_sub,mult_HB,mult_LB,div_R,div_Q;
    wire signed [31:0]A_Sign=A;
    wire signed [31:0]B_Sign=B;
    reg c_in;
    wire mult_rst=(((~op_code[3])&(~op_code[2])&op_code[1]&(~op_code[0])) || ((~op_code[3])&(~op_code[2])&op_code[1]&op_code[0]));
    wire div_rst=(((~op_code[3])&op_code[2]&(~op_code[1])&(~op_code[0])) || ((~op_code[3])&op_code[2]&(~op_code[1])&op_code[0]));
    assign BT=ALU_out[0];

    carry_look_ahead ADD(add_sub,A,B,c_in);
    Booths_mult MULT({mult_HB,mult_LB},mult_rst,A,B);
    non_rest_div DIV(div_R,div_Q,A,B,div_rst);

    // logic to select among immediate values and rs2
    always@(*) begin
        case (IRMUX)
            2'b00 : B=immS;
            2'b01 : B=immI;
            2'b10 : B=rs2;
            default: B=rs2;
        endcase
    end

    always@(*) begin
        ALU_out=32'd0; // default assignment 
        c_in=1'b0;     // to avoid latch
        JT=32'd0;
        case (op_code)
            4'b0000 : begin
                c_in = 1'b0;
                ALU_out = add_sub; // Addition
                if(instr==7'b1100111) JT=add_sub;
                else JT=32'd0;
            end 
            4'b0001 : begin
                c_in = 1'b1;
                ALU_out = add_sub; // Subtraction
            end
            4'b0010 : ALU_out = mult_HB; // Upper half bits of multiplication result.
            4'b0011 : ALU_out = mult_LB; // Lower half bits of multiplication result.
            4'b0100 : ALU_out = div_Q; // Quotient 
            4'b0101 : ALU_out = div_R; // Reminder
            4'b0110 : ALU_out = A & B; // AND
            4'b0111 : ALU_out = A | B; // OR
            4'b1000 : ALU_out = A ^ B; // XOR
            4'b1001 : ALU_out = A << B; // Left Shift
            4'b1010 : ALU_out = A >> B; // Right shift Logical
            4'b1011 : ALU_out = A >>> B_Sign; // Right Shift Arithmetic
            4'b1100 : begin // Set Less Than Signed
                ALU_out[31:1] = 31'd0;
                ALU_out[0] = (A_Sign<B_Sign)?1'b1:1'b0;
            end 
            4'b1101 : begin // Set Less Than Unsigned  
                ALU_out[31:1] = 31'd0;
                ALU_out[0] = (A<B)?1'b1:1'b0;
            end
            4'b1110 : begin // Equal or not
                ALU_out[31:1] = 31'd0;
                ALU_out[0] = (A==B)?1'b1:1'b0;
            end
            default :;
        endcase
    end
endmodule

// controller(FSM) for processor
module controller(
    output [3:0]ALUOP,
    output reg WERF,MWR,BJ_sel,
    output reg [1:0]IRMUX,PCMUX,
    output reg [2:0]WBMUX,
    input [3:0]dec_aluop,
    input [6:0]instr,
    input fsm_clk,fsm_rst,BT);

    parameter fetch = 3'b000,
    decode = 3'b001,
    read_src_regs = 3'b010,
    execute = 3'b011,
    rw_data_mem = 3'b100,
    write_back = 3'b101,
    call_nxt_pc = 3'b110;

    reg [2:0]present_state,next_state;

    // sequential logic for Present state
    always@(posedge fsm_clk) begin
        if(fsm_rst) present_state<=fetch;
        else present_state<=next_state;
    end

    // combinational logic for next state 
    always@(*) begin
        case (present_state)
            3'b000 : next_state=decode;
            3'b001 : begin
                if((instr==7'b0110011) || (instr==7'b0010011) ||
                (instr==7'b0000011) || (instr==7'b0100011) || 
                (instr==7'b1100011) || (instr==7'b1100111)) next_state=read_src_regs;
                else if(instr==7'b0010111) next_state=execute; // AUIPC instruction
                else if((instr==7'b0110111) || (instr==7'b1101111)) next_state=write_back; // LUI and JAL instruction
                else next_state=fetch; // default assignment
            end
            3'b010 : next_state=execute;
            3'b011 : begin
                if(instr==7'b0000011 || instr==7'b0100011) next_state=rw_data_mem; // Load and Store instructions
                else if(instr==7'b1100011) next_state=call_nxt_pc; // Branch instructions
                else if((instr==7'b1100111) ||(instr==7'b0110011) || 
                (instr==7'b0010011) ||(instr==7'b0110111) || (instr==7'b0010111)) next_state=write_back; // JALR,R,I,LUI and AUIPC instruction
                else next_state=fetch; // default assignment
            end
            3'b100 : begin
                if(instr==7'b0100011) next_state=call_nxt_pc;
                else next_state=write_back;
            end
            3'b101 : next_state=call_nxt_pc;
            3'b110 : next_state=fetch;
            default: next_state=fetch;
        endcase
    end

    // logic for output signals
    //logic for ALUOP
    assign ALUOP=(present_state==execute) ? dec_aluop : 4'bz;

    // logic for IRMUX
    always@(*) begin
        //if(present_state==execute) begin
            case (instr)
                7'b0110011 : IRMUX=2'b10; // takes rs2 value in R-type instruction
                7'b0010011 : IRMUX=2'b01; // takes immediate value in I-type instruction 
                7'b0000011 : IRMUX=2'b01; // takes imm value to calculate data mem address in Load instruction
                7'b0100011 : IRMUX=2'b00; // takes imm value to calculate data mem address in Store instruction
                7'b1100111 : IRMUX=2'b01; // takes imm value in JALR instruction
                default: IRMUX=2'b10;
            endcase
        //end
        //else IRMUX=2'b10;
    end

    // logic for MWR --> ~MWR=Write MWR=Read
    always@(*) begin
        if(present_state==rw_data_mem) begin
            if(instr==7'b0000011) MWR=1'b1; // Read operation (Load)
            else if(instr==7'b0100011) MWR=1'b0; // Write operation (Store)
            else MWR=1'bz;
        end
        else MWR=1'bz;
    end

    // logic for WBMUX signal
    always@(*) begin
        case (instr)
            7'b0110011 : WBMUX=3'b011; // write back alu o/p into reg file (R)
            7'b0010011 : WBMUX=3'b011; // write back alu o/p into reg file (I)
            7'b0000011 : WBMUX=3'b100; // write back data mem o/p into rge file (Load)
            7'b0100011 : WBMUX=3'bz; // no need to write back into reg file (Store)
            7'b1100011 : WBMUX=3'bz; // no need to write back into reg file (Branch)
            7'b1101111 : WBMUX=3'b000; // Write back pc+4 value into reg file (JAL)
            7'b1100111 : WBMUX=3'b000; // Write back pc+4 value into reg file (JALR)
            7'b0110111 : WBMUX=3'b001; // Write back IMMU into reg file (LUI)
            7'b0010111 : WBMUX=3'b010; // Write back pc+IMMU into reg file (AUIPC)
            default: WBMUX=3'b000;
        endcase
    end

    // logic for WERF signal
    always@(*) begin
        if(present_state==write_back) begin
            case (instr)
                7'b0110011 : WERF=1'b1; // write back alu o/p into reg file (R)
                7'b0010011 : WERF=1'b1; // write back alu o/p into reg file (I) 
                7'b0000011 : WERF=1'b1; // write back data mem o/p into rge file (Load)
                7'b0100011 : WERF=1'b0; // no need to write back into reg file (Store)
                7'b1100011 : WERF=1'b0; // no need to write back into reg file (Branch)
                7'b1101111 : WERF=1'b1; // Write back pc+4 value into reg file (JAL)
                7'b1100111 : WERF=1'b1; // Write back pc+4 value into reg file (JALR)
                7'b0110111 : WERF=1'b1; // Write back IMMU into reg file (LUI)
                7'b0010111 : WERF=1'b1; // Write back pc+IMMU into reg file (AUIPC)
                default: WERF=1'b0;
            endcase
        end
        else WERF=1'b0;
    end

    // logic for PCMUX
    always@(*) begin
        if(present_state==call_nxt_pc) begin
            case (instr)
                7'b1100011 : begin // Branch instruction
                    if(BT) PCMUX=2'b10; // pc=pc+IMMB BT=Branch target
                    else PCMUX=2'b01; // pc=pc+4
                    BJ_sel=1'b0; // selects sxt(imm) in Branch instruction
                end 
                7'b1101111 : begin // JAL instruction
                    PCMUX=2'b10; // pc=pc+IMMJ
                    BJ_sel=1'b1; // selects sxt(imm) in JAL instruction
                end
                7'b1100111 : begin
                    PCMUX=2'b11; // JT=rs1+IMMI pc={JT,1'b0}
                    BJ_sel=1'b0; // explicit assignment to avoid latch
                end
                7'b0110011, 7'b0010011, 7'b0000011, 7'b0100011, 7'b0110111: begin // R,I,Load,Store,LUI instructions
                    PCMUX=2'b01; // pc=pc+4
                    BJ_sel=1'b0; // explicit assignment to avoid latch
                end
                7'b0010111 : begin
                    PCMUX=2'b00; // AUIPC instruction
                    BJ_sel=1'b0; // explicit assignment to avoid latch
                end
                default: begin
                    PCMUX=2'b00; // pc=pc
                    BJ_sel=1'b0; // default assignment
                end
            endcase
        end
        else if((present_state==write_back) && (instr==7'b1101111 || instr==7'b1100111)) begin
            PCMUX=2'b01; // pc=pc+4 (JAL and JALR)
            BJ_sel=1'b0; // default assignment
        end
        else begin
            PCMUX=2'b00; // pc=pc
            BJ_sel=1'b0; // default assignment
        end
    end
endmodule

// Instruction decoder of processor
module instruction_dec(
    output reg [3:0]ALUOP,
    output reg [11:0]imm_bus,
    output reg [19:0]imm_bus_UJ,
    output reg [4:0]source_addr1,source_addr2,destination_addr,
    input [31:0]instr,
    input dec_clk);

    // Logic to decode ALUOP signal
    always@(*) begin
        case (instr[6:0]) // opcode
            7'b0110011 : begin // R-type instr
                case (instr[14:12]) // func3
                    3'b000 : begin
                        // instr[31:25] = func7
                        if(instr[31:25]==7'b0000000) ALUOP=4'd0; // ADD
                        else if(instr[31:25]==7'b0100000) ALUOP=4'd1; // SUB
                        else if(instr[31:25]==7'b0010000) ALUOP=4'd2; // MULT
                        else if(instr[31:25]==7'b0001000) ALUOP=4'd3; // DIV
                        else ALUOP=4'd0; // default assignment
                    end
                    3'b001 : ALUOP=4'd9; // Shift Left Logical
                    3'b010 : ALUOP=4'd12; // Set Less Than
                    3'b011 : ALUOP=4'd13; // Set Less Than (U)
                    3'b100 : ALUOP=4'd8; // XOR
                    3'b101 : begin
                        if(instr[31:25]==7'b0000000) ALUOP=4'd10; // Shift Right Logical
                        else if(instr[31:25]==7'b0100000) ALUOP=4'd11; // Shift Right Arith 
                        else ALUOP=4'd0; // default assignment
                    end
                    3'b110 : ALUOP=4'd7; // OR
                    3'b111 : ALUOP=4'd6; // AND
                endcase
            end 
            7'b0010011 : begin // I-type instr
                case (instr[14:12]) // func3
                    3'b000 : ALUOP=4'd0; // ADD_Imm
                    3'b001 : ALUOP=4'd9; // Shift Left Logical Imm
                    3'b010 : ALUOP=4'd12; // Set Less Than Imm
                    3'b011 : ALUOP=4'd13; // Set Less Than Imm (U)
                    3'b100 : ALUOP=4'd8; // XOR Imm
                    3'b101 : begin
                        if(instr[31:25]==7'b0000000) ALUOP=4'd10; // Shift Right Logical Imm
                        else if(instr[31:25]==7'b0100000) ALUOP=4'd11; // Shift Right Arith Imm
                        else ALUOP=4'd0; // default assignment
                    end  
                    3'b110 : ALUOP=4'd7; // OR Imm
                    3'b111 : ALUOP=4'd6; // AND Imm
                endcase
            end
            7'b0000011 : ALUOP=4'd0; // I-type Load instr
            7'b0100011 : ALUOP=4'd0; // Store instr
            7'b1100011 : begin // B-Type instr
                case (instr[14:12])
                    3'b000 : ALUOP=4'd14; // Equal
                    3'b001 : ALUOP=4'd12; // Less Than(S)
                    3'b110 : ALUOP=4'd13; // Less Than(U)
                    default : ALUOP=4'd14;
                endcase
            end
            7'b1100111 : ALUOP=4'd0; // JT=rs1+immI (JALR)
            default : ALUOP=4'd0;
        endcase
    end

    // Logic to decode immediate bus and address of source and destination registers
    always@(posedge dec_clk) begin
        case (instr[6:0]) // opcode
            7'b0110011 : begin // R-type instr
                destination_addr<=instr[11:7];
                source_addr1<=instr[19:15];
                source_addr2<=instr[24:20];
                imm_bus<=12'd0;
                imm_bus_UJ<=20'd0;
            end 
            7'b0010011 : begin // I-type instr
                destination_addr<=instr[11:7];
                source_addr1<=instr[19:15];
                source_addr2<=5'bz;
                imm_bus<=instr[31:20];
                imm_bus_UJ<=20'd0;
            end
            7'b0000011 : begin // Load instr
                destination_addr<=instr[11:7];
                source_addr1<=instr[19:15];
                source_addr2<=5'bz;
                imm_bus<=instr[31:20];
                imm_bus_UJ<=20'd0;
            end
            7'b0100011 : begin // Store instr
                destination_addr<=5'bz;            
                source_addr1<=instr[19:15];
                source_addr2<=instr[24:20];
                imm_bus<={instr[31:25],instr[11:7]};
                imm_bus_UJ<=20'd0;
            end
            7'b1100011 : begin // B-type instr
                imm_bus<={instr[31],instr[7],instr[30:25],instr[11:8]};
                destination_addr<=5'bz;
                source_addr1<=instr[19:15];
                source_addr2<=instr[24:20];
                imm_bus_UJ<=20'd0;
            end
            7'b1101111 : begin // Jump and Link (JAL)
                {source_addr1,source_addr2}<=10'bz;
                imm_bus<=12'd0;
                imm_bus_UJ<={instr[31],instr[19:12],instr[20],instr[30:21]};
                destination_addr<=instr[11:7];
            end
            7'b1100111 : begin // Jump and Link reg (JALR)
                {source_addr1,source_addr2}<=10'bz;
                imm_bus<=12'd0;
                imm_bus_UJ<={instr[31],instr[19:12],instr[20],instr[30:21]};
                destination_addr<=instr[11:7];
            end
            7'b0110111 : begin // LUI
                {source_addr1,source_addr2}<=10'bz;
                imm_bus<=12'd0;
                imm_bus_UJ<=instr[31:12];
                destination_addr<=instr[11:7];
            end
            7'b0010111 : begin // AUIPC
                {source_addr1,source_addr2}<=10'bz;
                imm_bus<=12'd0;
                imm_bus_UJ<=instr[31:12];
                destination_addr<=instr[11:7];
            end
            default : begin
                destination_addr<=instr[11:7];
                source_addr1<=instr[19:15];
                source_addr2<=instr[24:20];
                imm_bus<=12'd0;
                imm_bus_UJ<=20'bz;
            end
        endcase
    end
endmodule

//64 Bytes Data Memory
module data_mem(
    output reg [31:0] data_out,
    input data_mem_rst,data_mem_clk,
    input write_read, // logic-1 --> Read operation, logic-0 --> Write operation
    input [31:0] addr,
    input [31:0] data_in);

    reg [31:0]mem[63:0]; 
    integer i;

    always@(posedge data_mem_clk) begin
        if(data_mem_rst) begin
            for(i=0;i<64;i=i+1) mem[i]<=32'd0;
        end
        else begin
            if(!write_read) begin
                mem[addr]<=data_in;
                data_out<=32'bz;
            end
            else data_out<=mem[addr];
        end
    end
endmodule

// Instruction memory to store all instructions to be executed
module instruction_mem(
    output reg [31:0]instruction,
    input [31:0]addr);

    reg [31:0]mem[63:0];

    // desired instruction to be preloaded 
    initial begin
        mem[0]=32'b000011001000_01001_000_01001_0010011; // I instruction (rs1)
        mem[4]=32'b001111101000_01100_000_01100_0010011; // I instruction (rs2)
        mem[8]=32'b0000000_01100_01001_000_01011_0110011; // R instruction (rd=rs1+rs2)
    end

    always@(*) instruction=mem[addr];
endmodule

// Register file to store operands and other data
module reg_file(
    output reg [31:0] rs1,rs2,
    input reg_file_rst,reg_file_clk,write_enb,
    input [31:0] write_data,
    input [4:0] write_addr,read_addr1,read_addr2);
    
    reg [31:0]mem[31:0]; // memory cells
    integer i;

    always@(mem[0]) mem[0] = 32'd0; // hardwired to zero
   
    // Synchronous write operation
    always@(posedge reg_file_clk) begin
        if(reg_file_rst) begin
            for(i=1;i<32;i=i+1) mem[i] <= 32'd0;
        end
        else begin
            if(write_enb) mem[write_addr] <= write_data;
            else mem[write_addr] <= mem[write_addr];
        end
    end

    // Asynchronous read operation
    always@(*) begin
        rs1 = mem[read_addr1];
        rs2 = mem[read_addr2];
    end
endmodule

// Program counter used to point to the address of instruction memory
module prgm_cntr(
    output reg [31:0]pc,
    input [31:0]immB,immJ,
    input [30:0]JT,
    input [1:0]PCMUX,
    input BJ_sel,pc_clk,pc_rst);

    always@(posedge pc_clk) begin
        if(pc_rst) pc<=32'd0;
        else begin
            case (PCMUX)
                2'b00 : pc<=pc;
                2'b01 : pc<=pc+4;
                2'b10 : begin
                    if(!BJ_sel) pc<=pc+immB;
                    else pc<=pc+immJ;
                end
                2'b11 : pc<={JT,1'b0};
            endcase
        end
    end
endmodule

// Immediate value generating block
module imm_gen(
    output reg [31:0]immB,immJ,immI,immS,immU,
    input [11:0]imm1,
    input [19:0]imm2,
    input [6:0]opcode);

    always@(*) begin
        immB=32'd0;
        immJ=32'd0;
        immI=32'd0;
        immS=32'd0;
        immU=32'd0;
        case (opcode)
            7'b0010011, 7'b0000011, 7'b1100111  : immI={{20{imm1[11]}},imm1}; // I, Load, JALR-type instruction
            7'b0100011 : immS={{20{imm1[11]}},imm1}; // Store instruction
            7'b1100011 : immB={{19{imm1[11]}},imm1,1'b0}; // Branch instruction
            7'b1101111 : immJ={{11{imm2[19]}},imm2,1'b0}; // JAL instruction
            7'b0110111, 7'b0010111 : immU={imm2,12'd0}; // LUI instruction 
            default : ; // to avoid violation
        endcase
    end
endmodule

// Carry Look Ahead Adder/Subtractor
module carry_look_ahead(
    output reg [31:0]result,
    input [31:0]a,b,
    input cin);

    reg [31:0]p,g,r,y,c_o,B,temp1,temp2;

    integer i,j,k;
    always@(*) begin
        if(cin) begin
            if(a>b) begin
                temp1=a;
                temp2=b;
            end
            else begin
                temp1=b;
                temp2=a;
            end
        end
        else begin
            temp1=a;
            temp2=b;
        end

        for(k=0;k<32;k=k+1) begin
            // 2's complement of b.
            B[k]=temp2[k]^cin; // cin = 0 --> addition  cin = 1 --> subtraction
            // carry generate and propogation generate.
            p[k]=temp1[k]^B[k];
            g[k]=temp1[k]&B[k];
        end
        r={g[30:0],cin};
        // Logic for carry generator.
        for(k=0;k<=31;k=k+1) begin
            c_o[k]=g[k];
            for(i=0;i<=k;i=i+1) begin
                y[i]=r[i];
                for(j=i;j<=k;j=j+1) begin
                    y[i]=y[i]&p[j];
                end
                c_o[k]=c_o[k]|y[i];
            end
        end
        // iteration for sum signal.
        result[0]=p[0]^cin;
        for(k=1;k<32;k=k+1) result[k]=p[k]^c_o[k-1];
    end
endmodule

// Booth's Algorithm Multiplier
module Booths_mult(
    output reg [63:0]mult,
  input enb,
  input [31:0]M,Q);  // M-->Multiplicand Q-->Multiplier
  
  reg [5:0]cnt1,cnt2;
  reg [63:0]temp1,temp2;
  reg [31:0]m;
  reg [32:0]q;
  integer i;

  always@(*) m=(~M)+1'b1;

  always@(*) begin
    q={1'b0,{Q}};
    temp1=64'd0;
    mult=64'd0;
    if(!enb) begin
      {cnt1,cnt2}=0;
      {temp1,temp2}=0;
    end
    else begin
      for(i=32;i>=0;i=i-1) begin
        if(i==0) begin
          if((q[i]^1'b0)==1) temp1={{32{m[31]}},m};
          else temp1=temp1;
          temp2=temp2+temp1;
          {cnt1,cnt2}=0;
        end
        else begin
          if((q[i]^q[i-1])==1'b1) begin
            cnt2=i;
            cnt1=cnt1 + 1;
            if( (cnt1>0) && (cnt1%2==1)) temp1={32'b00000000,M}<<cnt2;
            else if( (cnt1>0) && (cnt1%2==0)) temp1={{32{m[31]}},m}<<cnt2;
            temp2=temp2+temp1;
          end
          else begin
            cnt1=cnt1;
            cnt2=cnt2;
            temp2=temp2;
          end
        end
        if((Q[0]==1'b1) && (i==0)) mult=temp2;
        else if((Q[0]==1'b0) && (i==1)) mult=temp2;
      end
    end
  end
endmodule

// Non-Restoring Division Algorithm
module non_rest_div(
    output reg [31:0]R,Q,
    input [31:0]a,b, // a--> Dividend b--> Divisor
    input rst);

    reg [32:0]acc;  // Accumulator
    reg [31:0]reg1,reg2,B,q;
    integer n;

    always@(*) begin
        if(a>b) begin
            reg1=a;
            reg2=b;
        end
        else begin
            reg1=b;
            reg2=a;
        end
        B=(~reg2)+1'b1;
        if(!rst) begin
            acc=0;
            q=reg1;
            {R,Q}=64'bz;
        end
        else begin
            for(n=32;n>=0;n=n-1) begin
                if(n!=0) begin
                    {acc,q}={acc,q}<<1;
                    if(acc[32]==0) acc=acc+{B[31],B};
                    else acc=acc+{1'b0,reg2};
                    if(acc[32]==0) q[0]=1'b1;
                    else q[0]=1'b0;
                end
                else begin
                    if(acc[32]==1) acc=acc+{1'b0,reg2};
                    else acc=acc;
                    R=acc[31:0];
                    Q=q;
                end
            end
        end
    end
endmodule

module risc_tb;
wire [31:0]pc,instr,ALUOUT;
wire [4:0]RS1_addr,RS2_addr,RD_addr;
wire [31:0]Jump_target;
wire Branch_target,MWR,WERF;
wire [31:0]data_mem_out,write_data;
wire [2:0]WBMUX;
wire [3:0]ALUOP;
wire [1:0]IRMUX,PCMUX;
reg clk,rst;

risc_v DUT(pc,instr,ALUOUT,RS1_addr,RS2_addr,RD_addr,Jump_target,Branch_target,
MWR,WERF,data_mem_out,write_data,WBMUX,ALUOP,IRMUX,PCMUX,clk,rst);

always #5 clk=~clk;

initial begin
    $dumpfile("risc_tb.vcd");
    $dumpvars(0,risc_tb);
    $monitor("t=%g clk=%b rst=%b PCMUX=%b pc=%d instruction=%b rs1_addr=%b rs2_addr=%b rd_addr=%b ALU_op=%b ALUOUT=%b IRMUX=%b JT=%b BT=%b data_mem_out=%d MWR=%b WBMUX=%b WERF=%b Write_data=%d",
    $time,clk,rst,PCMUX,pc,instr,RS1_addr,RS2_addr,RD_addr,ALUOP,ALUOUT,IRMUX,Jump_target,Branch_target,data_mem_out,MWR,WBMUX,WERF,write_data);
    {clk,rst}=2'b01;
    @(negedge clk) rst=1'b0;
    #500 $finish;
end
endmodule