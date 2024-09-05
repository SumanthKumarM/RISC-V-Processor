// Test bench 
`include "risc.v"

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
