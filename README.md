# RISC-V-Processor
This is 32-bit processor based on RISC-V Instruction Set Architecture.  
RISC-V ISA has 6 types of instructions :
1) R-type : Register to register instruction
2) I-type : Immediate type
3) Store instruction : Writes data into data memory
                                        4) Load instruction : Reads data from data memory
                                        5) Branch type : updates program counter based on conditions
                                        6) JAL type : Jump and Link instruction
                                        7) JALR type : Jump and Link Register 
                                        8) LUI type : Load upper immediate 
                                        9) AUIPC type : Add Upper Immediate to Program Counter
This processor has been designed to support all the above mentioned instructions. ALU data path has been designed in this processor in such a way that the processor accepts every type of instruction in RISC-V ISA and carries away the process according to the given instruction. 
Assembler converts the assembly language into its corresponding instruction in binary format which is understandable by processor. In this project instructions are given in binary format according to the command we want to execute.
There are a total of 8 modules included in this project. They are:
1) Arithmetic Logic Unit (ALU) : This is one of the most important component of any processor that performs arithmetic and logic operations based on the instruction given. To make this ALU time efficient and area efficient, the ALU has been designed using the following circuits:
               a) Carry Look Ahead Adder : This circuit is time efficient for word sizes like 32-bits. This adder is more time efficient that Ripple Carry adder as the output carry in every stage is dependant only on input carry. This circuit is used to carry out both 
                                           addition and subtraction. When input carry is given logic-0 the CLA performs addition and when input carry is logic-1 then the CLA performs subtraction.
               b) Booth's Multiplier : Booth's multiplication algorithm is used to implement binary multiplication with high time efficiency by skipping addition when zeros appear in multiplicand and it performs very less number of additions which also results in area 
                                       efficiency.
               c) Non-restoring division algorithm : This algorithm is used to implement time efficient and area efficient binary divider.
2) Program Counter : This component is used to update the address bus of instruction memory based on intsructions given and it helps to fecth instructions from instruction memory.
3) Instruction memory : This component stores all neccesary instructions needed to perform a certain task (for example to add two numbers and store in a register/sorting given array) and outputs those instructions based on the address given by program counter.
4) Instruction decoder : This is also one of the main components which decode the instruction fetched from instruction memory and decodes that instruction into opcode which specifies the type of instruction (like R,I-type etc), adresses of source registers and destinations registers, func3 and func7 which specify the type of operation performed by ALU, immediate bus used to produce different immediate values based on the different instructions.
5) Register file : This is a component used to store all operands involved in processing of an instruction. All operands used in a task are stored and loaded from this register file into ALU.
6) Controller : This is one main components whichh is an FSM used to control the processor actions by outputing neccesary signals based on the different instructions. Basically is communicates with each and every component of processor and tells them what to do in each and every instruction. There are a total of 7 states this controller undergoes and produces neccesary signals based on those states and instruction. Those states are :
               a) Fetch state : fetches the instruction from instruction memory into instrucion decoder.
               b) Decode state : decodes the type of instruction.
               c) read_src_regs : based on the decoded instruction, processor reads the contents of register file based on read address.
               d) execute : controller tells ALU what type of operation it should perform.
               e) rw_data_mem : this state tells data memory to read or write data.
               f) write_back : this state tells register file to perform write operation based on the write address decoded from instruction in decode state.
               g) call_nxt_pc : this state tells program counter to update its value based onn the instruction.
7) Data memory : Used in store instruction to read value from register file and in load instruction to write a value into register file. Address of data memory is calculated by ALU based on the instruction decoded.
8) Immediate generator : This component is used to generate differnt immediate values in I,store,load,JAL,JALR,LUI,AUIPC intsructions.
