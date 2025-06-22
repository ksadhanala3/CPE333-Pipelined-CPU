`timescale 1ns / 1ps

// 
////////////////////////////////////////////////////////////////////////////
typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic WE2;
    logic RDEN2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic mem_type_sign;  //sign, size
    logic [1:0] mem_type_size; // should i break up mem type
    logic [31:0] pc;
    logic [2:0] pcSource;
    logic [1:0] alu_srcA;
    logic [1:0] alu_srcB;
    logic [31:0] ir;
    logic [31:0] rs1;
    logic [31:0] rs2;
    logic [31:0] jal;
    logic [31:0] jalr;
    logic [31:0] branch;
    // logic PCWrite;
    // logic RDEN1;
    // logic reset DO I ACTUALLY NEED THIS YET
    //add cntrl signals from FSM
} instr_t;

typedef struct packed{
    logic [31:0] I_type;
    logic [31:0] S_type;
    logic [31:0] B_type;
    logic [31:0] U_type;
    logic [31:0] J_type;
} imm;

module OTTER_CPU(
    input clk,
    input RST,
    //input intr,
    input [31:0] iobus_in,
    output [31:0] iobus_out,
    output [31:0] iobus_addr,
    output iobus_wr
    );
    //wires
    wire [31:0] pc;
    wire [31:0] data;
    wire [31:0] ir;
    wire [31:0] I_type, B_type, J_type, U_type, S_type;
    wire [31:0] jal;
    wire [31:0] branch;
    wire [31:0] jalr;
    wire [1:0] rf_wr_sel;
    wire [3:0] alu_fun;
    wire [1:0] alu_srcA;
    wire [1:0] alu_srcB;
    wire [2:0] PCSource;
    wire [31:0] A,B;
    wire [31:0] rs1, rs2;
    wire [31:0] dout2, wd_out;
    wire PCWrite;
    wire regWrite;
    wire WE2;
    wire RDEN1;
    wire RDEN2;
    wire reset;
    wire ERR;
//    wire [31:0] mtvec, mepc, csr_RD;
//    wire int_taken, mret_exec, csr_WE, MIE, MPIE;
    wire EQ, LT, LTU;
    reg [31:0] result;
    reg Zero;
    reg PCStall = 0;
    wire [31:0] stallMUX;
    
    reg [31:0] FWD_rs1;
    reg [31:0] FWD_rs2;
    reg rs1_flag;
    reg rs2_flag;
//    reg TooRAW;
    
    logic [31:0] de_ex_opA_wire;
    logic [31:0] de_ex_opB_wire;
    
    //linked input-output signals
    assign iobus_addr = result;
    assign iobus_out = rs2;
    assign reset = RST;
    //==== Instruction Fetch ===========================================

     logic [31:0] if_de_pc, if_de_PCPlus4;
     
     always_ff @(posedge clk) begin
                if_de_pc <= pc;
                if_de_PCPlus4 <= pc +4;  
     end
     
     assign PCWrite = 1'b1; 	//Hardwired high, assuming no hazards
     assign RDEN1 = 1'b1; 	//Fetch new instruction every cycle
     
     
     
//     TwoMux Stall_MUX (
//        .SEL(PCStall),
//        .ONE(stallMUX),
//        .TWO(pc),
//        .OUT(data)
//        );
        
     //PC MUX
    PC_MUX PC_MUX(
        .PC_SOURCE        (PCSource),
        .PC_OUT_PLUS_FOUR         (pc + 4),
        .JALR         (jalr),
        .BRANCH         (branch),
        .JAL         (jal),
        .PC         (pc),
//        .D4         (mtvec),
//        .D5         (mepc),
        .PC_MUX_OUT      (data)    
        );
        
    //PC register    
    PC_REG PCREG(
        .IN        (data),
        .CLK            (clk),
        .RST            (reset),
        .D             (PCWrite), //put strcut version?? but i dont have one for fetch
        .OUT       (pc)    
        );
        
   
    
//==== Instruction Decode ===========================================
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;

    instr_t decode_instr, execute_instr, mem_instr, wb_instr;
    imm decode_imm, execute_imm, mem_imm, wb_imm;
    
    // decode struct
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(ir[6:0]);
    assign decode_instr.rs1_addr=ir[19:15];
    assign decode_instr.rs2_addr=ir[24:20];
    assign decode_instr.rd_addr=ir[11:7];
    assign decode_instr.opcode=OPCODE;
    assign decode_instr.alu_fun = alu_fun;
    assign decode_instr.WE2 = WE2;
    assign decode_instr.RDEN2 = RDEN2;
    assign decode_instr.regWrite = regWrite;
    assign decode_instr.rf_wr_sel = rf_wr_sel;
    assign decode_instr.mem_type_sign = ir[14];  //sign, size
    assign decode_instr.mem_type_size = ir[13:12];
    assign decode_instr.pc = if_de_pc;
    assign decode_instr.pcSource = PCSource;
    assign decode_instr.alu_srcA = alu_srcA;
    assign decode_instr.alu_srcB = alu_srcB;
    assign decode_instr.ir = ir;
    
//    assign decode_instr.rs1 = rs1;
//    assign decode_instr.rs2 = rs2;
    
    
    
    // imm
    assign decode_imm.I_type = I_type;
    assign decode_imm.J_type = J_type;
    assign decode_imm.B_type = B_type;
    assign decode_imm.U_type = U_type;
    assign decode_imm.S_type = S_type;

    
    always_ff@(posedge clk) begin
        execute_instr <= decode_instr;
        execute_imm <= decode_imm;
        if(rs1_flag)
            de_ex_opA_wire <= FWD_rs1;
        else
            de_ex_opA_wire <= de_ex_opA;  //if not forwarding, pass on to register
        if(rs2_flag)
            de_ex_opB_wire <= FWD_rs2;
        else
            de_ex_opB_wire <= de_ex_opB;
        
    end
    
    
    CU_DCDR  my_cu_dcdr(
        .br_eq          (EQ), 
        .br_lt          (LT), 
        .br_ltu         (LTU),
        .opcode         (OPCODE),
        .func7          (ir[30]),
        .func3          (ir[14:12]),
        .Stall          (PCStall),
 //       .INT_TAKEN      (int_taken),
        .alu_fun        (alu_fun),
        .pcSource       (PCSource),
        .alu_srcA       (alu_srcA),
        .alu_srcB       (alu_srcB),
        .rf_wr_sel      (rf_wr_sel), //reg MUX select
        .pcWrite        (), //no need for struct?
        .regWrite       (regWrite),
        .memWE2         (WE2),
        .memRDEN1       (),
        .memRDEN2       (decode_instr.RDEN2)       
        );
        
    
        
      //alu A MUX (2 input)
    FourMux my_muxA(
       .SEL   (decode_instr.alu_srcA), 
       .ZERO    (rs1), 
       .ONE    (U_type),
       .TWO  (0),
       .THREE    (~decode_instr.rs1), 
       .OUT (de_ex_opA) ); //output of alu source mux A
      
    
    //alu B MUX
    FourMux my_muxB(
       .SEL   (decode_instr.alu_srcB), 
       .ZERO    (rs2), 
       .ONE    (decode_imm.I_type), 
       .TWO    (S_type), 
       .THREE    (decode_instr.pc),
       //.D4    (csr_RD),
       .OUT (de_ex_opB) ); 
       
        //imm gen module
     assign I_type = {{21{ir[31]}}, ir[30:25], ir[24:20]};
     assign S_type = {{21{ir[31]}}, ir[30:25], ir[11:7]};
     assign B_type = {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0};
     assign U_type = {ir[31:12], 12'h000};
     assign J_type = {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};
     
	
//==== Execute ======================================================
     logic [31:0] ex_mem_rs2;
     logic [31:0] ex_mem_aluRes;
     logic [31:0] ex_mem_opA;
     logic [31:0] ex_mem_opB;
//     logic [31:0] opA_forwarded;
//     logic [31:0] opB_forwarded;
    
//    assign execute_instr.regWrite = regWrite;
//    assign execute_instr.WE2 = WE2;
//     assign jal = execute_instr.jal;
     
     always_ff@(posedge clk) begin
        mem_instr <= execute_instr;
        de_ex_rs2 <= rs2; //should i do this???
        ex_mem_aluRes <= result;
        mem_imm <= execute_imm;
     end
          
        
        
     
     
     always_comb begin
      //control hazard handling
//        if(mem_instr.pcSource == 3'b010 || mem_instr.pcSource == 3'b011 || mem_instr.pcSource == 3'b001) //branch, jal, jalr
//        begin
//            regWrite = 1'b0;
//            WE2 = 1'b0;
//            decode_instr.regWrite <= 1'b0;
//            decode_instr.WE2 <= 1'b0;
//        end
//        else
//        begin
//            execute_instr.regWrite <= 1'b1;
//            execute_instr.WE2 <= 1'b1;
//            decode_instr.regWrite <= 1'b1;
//            decode_instr.WE2 <= 1'b1;
//        end if(PCStall)
         if(execute_instr.rs1_addr == mem_instr.rd_addr)  //forward to rs1
                begin
                    FWD_rs1 = result;   //ex_mem_aluRes
                    rs1_flag = 1'b1;
                end
                if(execute_instr.rs2_addr == mem_instr.rd_addr)  //forward to rs2
                begin
                    FWD_rs2 = result;  //ex_mem_aluRes
                    rs2_flag = 1'b1;
                end
         else
            if(execute_instr.rs1_addr == wb_instr.rd_addr)  //forward to rs1
                begin
                    FWD_rs1 = iobus_addr;
                end
                else
                    FWD_rs1 = rs1;
                if(execute_instr.rs2_addr == wb_instr.rd_addr)  //forward to rs2
                begin
                    FWD_rs2 = iobus_addr;
                end
                else
                    FWD_rs2 = rs2;

        
        
        
     //check for RAW and forward on load
        if(decode_instr.rs1_addr == execute_instr.rd_addr || decode_instr.rs2_addr == execute_instr.rd_addr)
        begin
            if(execute_instr.RDEN2 == 1'b1)  //if load instr before
            begin
                PCStall = 1'b1;  //stall pc
            end
            else
            begin
                PCStall = 1'b0;
            end
            
//            if(execute_instr.rs1_addr == mem_instr.rd_addr)  //forward to rs1
//            begin
//                execute_instr.rs1 = ex_mem_aluRes;
//            end
//            if(execute_instr.rs2_addr == mem_instr.rd_addr)  //forward to rs2
//            begin
//                execute_instr.rs2 = ex_mem_aluRes;
//            end
        end
        
        
     end
    
     
     // Creates a RISC-V ALU 
     // the ALU
     ALU  my_alu(
        .SRC_A          (de_ex_opA_wire), //pull data from register
        .SRC_B          (de_ex_opB_wire),
        .ALU_FUN    (execute_instr.alu_fun), //execute_instr??
        .RESULT     (result)
//        .Zero       (Zero)
    );
    

    // branch cond generator 
    // it only outputs pcSpurce
    BCG my_bcg(
        .RS1    (de_ex_opA_wire),
        .RS2    (de_ex_opB_wire),
        .BR_EQ  (EQ),
        .BR_LT  (LT),
        .BR_LTU (LTU)
    );
     
     //branch module
     assign jal = execute_instr.pc + decode_imm.J_type;
     assign jalr = decode_imm.I_type + execute_instr.rs1;
     assign branch = execute_instr.pc + decode_imm.B_type;
     


//==== Memory ====================================================== 
    assign iobus_addr = ex_mem_aluRes;
//    assign iobus_out = ex_mem_rs2;
    logic [31:0] wb_dout2;
    
    always_ff@(posedge clk) begin
        wb_instr <= mem_instr;
        wb_imm <= mem_imm;
        ex_mem_rs2 <= de_ex_rs2;
        wb_dout2 <= dout2;
        
//        if(TooRAW)
//            if(rs1_flag)
//            de_ex_opA_wire <= FWD_rs1;
//        if(rs2_flag)
//            de_ex_opA_wire <= FWD_rs2; 
        
            
    end
    
    
//    always_comb begin
        
//        //check for RAW and forward
//        if(execute_instr.rs1_addr == wb_instr.rd_addr || decode_instr.rs2_addr == mem_instr.rd_addr)
//        begin
//            TooRAW = 1'b1;
//        end
//        else
//            TooRAW = 1'b0;
//    end
     
     //memory
   Memory OTTER_MEMORY (
        .MEM_CLK    (clk),
        .MEM_RDEN1  (RDEN1),
        .MEM_RDEN2  (mem_instr.RDEN2),
        .MEM_WE2    (mem_instr.WE2),
        .MEM_ADDR1  (pc[15:2]),    //14 bit signal
        .MEM_ADDR2  (ex_mem_aluRes),
        .MEM_DIN2   (ex_mem_rs2),
        .MEM_SIZE   (mem_instr.mem_type_size),        //ir [13:12
        .MEM_SIGN   (mem_instr.mem_type_sign),         //ir[14]
        .IO_IN      (iobus_in),
        .IO_WR      (iobus_wr),                //unused
        .MEM_DOUT1  (ir),
        .MEM_DOUT2  (dout2) );  
        
//==== Write Back ==================================================
    logic [31:0] mem_wb_aluRes;
    
    always_ff@(posedge clk) begin
        mem_wb_aluRes <= ex_mem_aluRes;
    end
    
     //REG FILE MUX
    FourMux my_muxREG(
       .SEL   (wb_instr.rf_wr_sel), 
       .ZERO    (wb_instr.pc + 4), 
       //.ONE    (csr_RD), 
       .TWO    (wb_dout2), 
       .THREE    (mem_wb_aluRes),
       .OUT (wd_out) ); 
       
    //REG FILE
    REG_FILE my_regfile (
        .WD             (wd_out),
        .CLK            (clk), 
        .EN             (mem_instr.regWrite),
        .ADR1           (decode_instr.rs1_addr),
        .ADR2           (decode_instr.rs2_addr),
        .WA             (mem_instr.rd_addr), //should this come from the wb struct?
        .RS1            (rs1), 
        .RS2            (rs2)  
        );

endmodule