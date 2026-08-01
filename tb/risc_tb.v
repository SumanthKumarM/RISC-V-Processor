// Test bench
// Minimal smoke run: streams a short program into the instruction memory
// through the load port, then releases the core and watches it execute.
// (Phase-2 adds the real directed / random testbenches.)
`include "risc.v"

module risc_tb;
wire [31:0] pc,instr,ALUOUT;
wire [4:0]  RS1_addr,RS2_addr,RD_addr;
wire [31:0] Jump_target;
wire        Branch_target,MWR,WERF;
wire [31:0] data_mem_out,write_data;
wire [2:0]  WBMUX;
wire [4:0]  ALUOP;
wire [1:0]  IRMUX,PCMUX;
reg         clk,rst;
reg         imem_load_en;
reg  [31:0] imem_load_data;

risc_v DUT(pc,instr,ALUOUT,RS1_addr,RS2_addr,RD_addr,Jump_target,Branch_target,
MWR,WERF,data_mem_out,write_data,WBMUX,ALUOP,IRMUX,PCMUX,clk,rst,
imem_load_en,imem_load_data);

always #5 clk=~clk;

// push one instruction word into the instruction memory
task send(input [31:0] w);
    begin imem_load_data = w; @(negedge clk); end
endtask

initial begin
    $dumpfile("risc_tb.vcd");
    $dumpvars(0,risc_tb);
    $monitor("t=%g clk=%b rst=%b PCMUX=%b pc=%d instruction=%b rs1_addr=%b rs2_addr=%b rd_addr=%b ALU_op=%b ALUOUT=%b IRMUX=%b JT=%b BT=%b data_mem_out=%d MWR=%b WBMUX=%b WERF=%b Write_data=%d",
    $time,clk,rst,PCMUX,pc,instr,RS1_addr,RS2_addr,RD_addr,ALUOP,ALUOUT,IRMUX,Jump_target,Branch_target,data_mem_out,MWR,WBMUX,WERF,write_data);

    clk = 1'b0; rst = 1'b1;
    imem_load_en = 1'b0; imem_load_data = 32'd0;

    // ---- load the program while the core is held in reset ----
    @(negedge clk);
    imem_load_en = 1'b1;
    // x9 = 200, x12 = 1000, x11 = x9 + x12
    send(32'b000011001000_01001_000_01001_0010011);    // addi x9,x9,200
    send(32'b001111101000_01100_000_01100_0010011);    // addi x12,x12,1000
    send(32'b0000000_01100_01001_000_01011_0110011);   // add  x11,x9,x12
    send(32'b0_000000_00000_00000_000_0000_0_1100011); // beq  x0,x0,0 -> halt
    imem_load_en = 1'b0;

    // ---- release the core ----
    @(negedge clk) rst = 1'b0;

    #500;
    $display("RESULT: x9=%0d x12=%0d x11=%0d",
        DUT.register_file.mem[9], DUT.register_file.mem[12], DUT.register_file.mem[11]);
    $finish;
end
endmodule
