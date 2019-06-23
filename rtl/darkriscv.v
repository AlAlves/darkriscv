/*
 * Copyright (c) 2018, Marcelo Samsoniuk
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

`timescale 1ns / 1ps

// opcodes

`define LUI     7'b0110111
`define AUIPC   7'b0010111
`define JAL     7'b1101111
`define JALR    7'b1100111
`define BCC     7'b1100011
`define LCC     7'b0000011
`define SCC     7'b0100011
`define MCC     7'b0010011
`define RCC     7'b0110011
`define FCC     7'b0001111
`define CCC     7'b1110011

// pipeline stages:
//
// 2-stages: core and memory in different clock edges result in less clock performance, but
// less losses when the program counter changes (pipeline flush = 1 clock). Works like a 4-stage
// pipeline and remember the 68040 clock scheme, with instruction per clock = 1.
// alternatively, it is possible work w/ 1 wait-state and 1 clock edge, but with a penalty in
// performance (instruction per clock = 0.5).
//
// 3-stage: core and memory in the same clock edge require one extra stage in the pipeline, but
// keep a good performance most of time (instruction per clock = 1). of course, read operations
// require 1 wait-state, which means sometimes the read performance is reduced.

`define __3STAGE__

module darkriscv
#(
    parameter [31:0] RESET_PC = 0,
    parameter [31:0] RESET_SP = 4096
) (
    input             CLK,   // clock
    input             RES,   // reset
    input             HLT,   // halt

    input      [31:0] IDATA, // instruction data bus
    output     [31:0] IADDR, // instruction addr bus

    input      [31:0] DATAI, // data bus (input)
    output     [31:0] DATAO, // data bus (output)
    output     [31:0] DADDR, // addr bus

    output     [ 3:0] BE,   // byte enable

    output            WR,    // write enable
    output            RD,    // read enable

`ifdef RISCV_FORMAL
    output reg        rvfi_valid = 1'b0,
    output reg [63:0] rvfi_order = 64'd0,
    output reg [31:0] rvfi_insn = 32'd0,
    output reg 	      rvfi_trap = 1'b0,
    output reg 	      rvfi_halt = 1'b0,
    output reg 	      rvfi_intr = 1'b0,
    output reg [1:0]  rvfi_mode = 2'b11,

	output reg [ 4:0] rvfi_rs1_addr,
	output reg [ 4:0] rvfi_rs2_addr,
	output reg [31:0] rvfi_rs1_rdata,
	output reg [31:0] rvfi_rs2_rdata,
	output reg [ 4:0] rvfi_rd_addr,
	output reg [31:0] rvfi_rd_wdata,

	output reg [31:0] rvfi_pc_rdata,
	output reg [31:0] rvfi_pc_wdata,

	output reg [31:0] rvfi_mem_addr,
	output reg [ 3:0] rvfi_mem_rmask,
	output reg [ 3:0] rvfi_mem_wmask,
	output reg [31:0] rvfi_mem_rdata,
	output reg [31:0] rvfi_mem_wdata,

	// output reg [63:0] rvfi_csr_mcycle_rmask,
	// output reg [63:0] rvfi_csr_mcycle_wmask,
	// output reg [63:0] rvfi_csr_mcycle_rdata,
	// output reg [63:0] rvfi_csr_mcycle_wdata,
    //
	// output reg [63:0] rvfi_csr_minstret_rmask,
	// output reg [63:0] rvfi_csr_minstret_wmask,
	// output reg [63:0] rvfi_csr_minstret_rdata,
	// output reg [63:0] rvfi_csr_minstret_wdata,
`endif

    output [3:0]  DEBUG      // old-school osciloscope based debug! :)
);

    // dummy 32-bit words w/ all-0s and all-1s:

    wire [31:0] ALL0  = 0;
    wire [31:0] ALL1  = -1;

    reg [1:0] FLUSH;  // flush instruction pipeline

    // IDATA is break apart as described in the RV32I specification

    reg [31:0] XIDATA;

    wire [6:0] OPCODE = FLUSH ? 0 : XIDATA[6:0];
    wire [4:0] DPTR   = XIDATA[11: 7];
    wire [2:0] FCT3   = XIDATA[14:12];
    wire [4:0] S1PTR  = XIDATA[19:15];
    wire [4:0] S2PTR  = XIDATA[24:20];
    wire [6:0] FCT7   = XIDATA[31:25];

    reg XLUI, XAUIPC, XJAL, XJALR, XBCC, XLCC, XSCC, XMCC, XRCC; //, XFCC, XCCC;

    always@(posedge CLK)
    begin
        XIDATA <= RES ? { ALL0[31:12], 5'd2, ALL0[6:0] } : HLT ? XIDATA : IDATA;

        XLUI   <= RES ? 0 : HLT ? XLUI   : IDATA[6:0]==`LUI;
        XAUIPC <= RES ? 0 : HLT ? XAUIPC : IDATA[6:0]==`AUIPC;
        XJAL   <= RES ? 0 : HLT ? XJAL   : IDATA[6:0]==`JAL;
        XJALR  <= RES ? 0 : HLT ? XJALR  : IDATA[6:0]==`JALR;

        XBCC   <= RES ? 0 : HLT ? XBCC   : IDATA[6:0]==`BCC;
        XLCC   <= RES ? 0 : HLT ? XLCC   : IDATA[6:0]==`LCC;
        XSCC   <= RES ? 0 : HLT ? XSCC   : IDATA[6:0]==`SCC;
        XMCC   <= RES ? 0 : HLT ? XMCC   : IDATA[6:0]==`MCC;

        XRCC   <= RES ? 0 : HLT ? XRCC   : IDATA[6:0]==`RCC;
        //XFCC   <= RES ? 0 : HLT ? XFCC   : IDATA[6:0]==`FCC;
        //XCCC   <= RES ? 0 : HLT ? XCCC   : IDATA[6:0]==`CCC;
    end

    // signal extended immediate, according to the instruction type:

    reg [31:0] SIMM;

    always@(posedge CLK)
    begin
        SIMM  <= RES ? 0 : HLT ? SIMM :
                 IDATA[6:0]==`SCC ? { IDATA[31] ? ALL1[31:12]:ALL0[31:12], IDATA[31:25],IDATA[11:7] } : // s-type
                 IDATA[6:0]==`BCC ? { IDATA[31] ? ALL1[31:13]:ALL0[31:13], IDATA[31],IDATA[7],IDATA[30:25],IDATA[11:8],ALL0[0] } : // b-type
                 IDATA[6:0]==`JAL ? { IDATA[31] ? ALL1[31:21]:ALL0[31:21], IDATA[31], IDATA[19:12], IDATA[20], IDATA[30:21], ALL0[0] } : // j-type
                 IDATA[6:0]==`LUI||
                 IDATA[6:0]==`AUIPC ? { IDATA[31:12], ALL0[11:0] } : // u-type
                                      { IDATA[31] ? ALL1[31:12]:ALL0[31:12], IDATA[31:20] }; // i-type
    end

    // non-signal extended immediate, according to the instruction type:

    reg [31:0] UIMM;

    always@(posedge CLK)
    begin
        UIMM  <= RES ? 0: HLT ? UIMM :
                 IDATA[6:0]==`SCC ? { ALL0[31:12], IDATA[31:25],IDATA[11:7] } : // s-type
                 IDATA[6:0]==`BCC ? { ALL0[31:13], IDATA[31],IDATA[7],IDATA[30:25],IDATA[11:8],ALL0[0] } : // b-type
                 IDATA[6:0]==`JAL ? { ALL0[31:21], IDATA[31], IDATA[19:12], IDATA[20], IDATA[30:21], ALL0[0] } : // j-type
                 IDATA[6:0]==`LUI||
                 IDATA[6:0]==`AUIPC ? { IDATA[31:12], ALL0[11:0] } : // u-type
                                      { ALL0[31:12], IDATA[31:20] }; // i-type
    end

    // main opcode decoder:

    wire    LUI = FLUSH ? 0 : XLUI;   // OPCODE==7'b0110111;
    wire  AUIPC = FLUSH ? 0 : XAUIPC; // OPCODE==7'b0010111;
    wire    JAL = FLUSH ? 0 : XJAL;   // OPCODE==7'b1101111;
    wire   JALR = FLUSH ? 0 : XJALR;  // OPCODE==7'b1100111;

    wire    BCC = FLUSH ? 0 : XBCC; // OPCODE==7'b1100011; //FCT3
    wire    LCC = FLUSH ? 0 : XLCC; // OPCODE==7'b0000011; //FCT3
    wire    SCC = FLUSH ? 0 : XSCC; // OPCODE==7'b0100011; //FCT3
    wire    MCC = FLUSH ? 0 : XMCC; // OPCODE==7'b0010011; //FCT3

    wire    RCC = FLUSH ? 0 : XRCC; // OPCODE==7'b0110011; //FCT3
    //wire    FCC = FLUSH ? 0 : XFCC; // OPCODE==7'b0001111; //FCT3
    //wire    CCC = FLUSH ? 0 : XCCC; // OPCODE==7'b1110011; //FCT3

`ifdef __3STAGE__
    reg [31:0] NXPC2;       // 32-bit program counter t+2
`endif
    reg [31:0] NXPC;        // 32-bit program counter t+1
    reg [31:0] PC;		    // 32-bit program counter t+0

    reg [31:0] REG1 [0:31];	// general-purpose 32x32-bit registers (s1)
    reg [31:0] REG2 [0:31];	// general-purpose 32x32-bit registers (s2)

    integer i;
    initial
    for(i=0;i!=32;i=i+1)
    begin
        REG1[i] = 0; // makes the simulation looks better!
        REG2[i] = 0; // makes the simulation looks better!
    end

    // source-1 and source-1 register selection

    wire signed   [31:0] S1REG = REG1[S1PTR];
    wire signed   [31:0] S2REG = REG2[S2PTR];

    wire          [31:0] U1REG = REG1[S1PTR];
    wire          [31:0] U2REG = REG2[S2PTR];

    // L-group of instructions (OPCODE==7'b0000011)

    wire [31:0] LDATA = FCT3==0||FCT3==4 ? ( DADDR[1:0]==3 ? { FCT3==0&&DATAI[31] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[31:24] } :
                                             DADDR[1:0]==2 ? { FCT3==0&&DATAI[23] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[23:16] } :
                                             DADDR[1:0]==1 ? { FCT3==0&&DATAI[15] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[15: 8] } :
                                                             { FCT3==0&&DATAI[ 7] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[ 7: 0] } ):
                        FCT3==1||FCT3==5 ? ( DADDR[1]==1   ? { FCT3==1&&DATAI[31] ? ALL1[31:16]:ALL0[31:16] , DATAI[31:16] } :
                                                             { FCT3==1&&DATAI[15] ? ALL1[31:16]:ALL0[31:16] , DATAI[15: 0] } ) :
                                             DATAI;

    // S-group of instructions (OPCODE==7'b0100011)

    wire [31:0] SDATA = FCT3==0 ? ( DADDR[1:0]==3 ? { U2REG[ 7: 0], ALL0 [23:0] } :
                                    DADDR[1:0]==2 ? { ALL0 [31:24], U2REG[ 7:0], ALL0[15:0] } :
                                    DADDR[1:0]==1 ? { ALL0 [31:16], U2REG[ 7:0], ALL0[7:0] } :
                                                    { ALL0 [31: 8], U2REG[ 7:0] } ) :
                        FCT3==1 ? ( DADDR[1]==1   ? { U2REG[15: 0], ALL0 [15:0] } :
                                                    { ALL0 [31:16], U2REG[15:0] } ) :
                                    U2REG;

    // C-group not implemented yet!

    wire [31:0] CDATA = 0;	// status register istructions not implemented yet

    // RM-group of instructions (OPCODEs==7'b0010011/7'b0110011), merged! src=immediate(M)/register(R)

    wire signed [31:0] S2REGX = XMCC ? SIMM : S2REG;
    wire        [31:0] U2REGX = XMCC ? UIMM : U2REG;

`ifdef MODEL_TECH
    wire [31:0] RMDATA_FCT3EQ5 = FCT7[5]==0||U1REG[31]==0 ? U1REG>>U2REGX[4:0] : // workaround for modelsim
                                -((-U1REG)>>U2REGX[4:0]);
`else
    wire [31:0] RMDATA_FCT3EQ5 = (FCT7[5] ? U1REG>>>U2REGX[4:0] : U1REG>>U2REGX[4:0]);
`endif
    wire [31:0] RMDATA = FCT3==0 ? (XRCC&&FCT7[5] ? U1REG-U2REGX : U1REG+S2REGX) :
                         FCT3==1 ? U1REG<<U2REGX[4:0] :
                         FCT3==2 ? S1REG<S2REGX?1:0 : // signed
                         FCT3==3 ? U1REG<U2REGX?1:0 : // unsigned
                         FCT3==5 ? RMDATA_FCT3EQ5 : // (FCT7[5] ? U1REG>>>U2REG[4:0] : U1REG>>U2REG[4:0]) :
                         FCT3==4 ? U1REG^S2REGX :
                         FCT3==6 ? U1REG|S2REGX :
                         FCT3==7 ? U1REG&S2REGX :
                                   0;

    // J/B-group of instructions (OPCODE==7'b1100011)

    wire BMUX       = BCC==1 && (
                          FCT3==4 ? S1REG< S2REG : // blt
                          FCT3==5 ? S1REG>=S2REG : // bge
                          FCT3==6 ? U1REG< U2REG : // bltu
                          FCT3==7 ? U1REG>=U2REG : // bgeu
                          FCT3==0 ? U1REG==U2REG : // beq
                          FCT3==1 ? U1REG!=U2REG : // bne
                                    0);

    wire        JREQ = (JAL||JALR||BMUX);
    wire [31:0] JVAL = SIMM + (JALR ? U1REG : PC);

    always@(posedge CLK)
    begin
`ifdef __3STAGE__
	    FLUSH <= RES ? 2 : HLT ? FLUSH :        // reset and halt
	                       FLUSH ? FLUSH-1 :
	                       (JAL||JALR||BMUX||RES) ? 2 : 0;  // flush the pipeline!
`else
        FLUSH <= RES ? 1 : HLT ? FLUSH :        // reset and halt
                       (JAL||JALR||BMUX||RES);  // flush the pipeline!
`endif

        REG1[DPTR] <=   RES ? RESET_SP  :        // reset sp
                       HLT ? REG1[DPTR] :        // halt
                     !DPTR ? 0 :                // x0 = 0, always!
                     AUIPC ? PC+SIMM :
                      JAL||
                      JALR ? NXPC :
                       LUI ? SIMM :
                       LCC ? LDATA :
                  MCC||RCC ? RMDATA:
                       //MCC ? MDATA :
                       //RCC ? RDATA :
                       //CCC ? CDATA :
                             REG1[DPTR];

        REG2[DPTR] <=   RES ? RESET_SP  :        // reset sp
                       HLT ? REG2[DPTR] :        // halt
                     !DPTR ? 0 :                // x0 = 0, always!
                     AUIPC ? PC+SIMM :
                      JAL||
                      JALR ? NXPC :
                       LUI ? SIMM :
                       LCC ? LDATA :
                  MCC||RCC ? RMDATA:
                       //MCC ? MDATA :
                       //RCC ? RDATA :
                       //CCC ? CDATA :
                             REG2[DPTR];

`ifdef __3STAGE__

        NXPC <= RES ? RESET_PC : HLT ? NXPC : NXPC2;

	    NXPC2 <=  RES ? RESET_PC : HLT ? NXPC2 :   // reset and halt
	                 JREQ ? JVAL :                    // jmp/bra
	                        NXPC2+4;                   // normal flow

`else
        NXPC <= RES ? RESET_PC : HLT ? NXPC :   // reset and halt
              JREQ ? JVAL :                   // jmp/bra
                     NXPC+4;                   // normal flow
`endif
        PC   <= RES ? RESET_PC : HLT ? PC : NXPC; // current program counter
    end

    // IO and memory interface

    assign DATAO = SDATA; // SCC ? SDATA : 0;
    assign DADDR = U1REG + SIMM; // (SCC||LCC) ? U1REG + SIMM : 0;
    assign RD = LCC;
    assign WR = SCC;

    // based in the Scc and Lcc
    assign BE = FCT3==0||FCT3==4 ? ( DADDR[1:0]==3 ? 4'b1000 : // sb/lb
                                     DADDR[1:0]==2 ? 4'b0100 :
                                     DADDR[1:0]==1 ? 4'b0010 :
                                                     4'b0001 ) :
                FCT3==1||FCT3==5 ? ( DADDR[1]==1   ? 4'b1100 : // sh/lh
                                                     4'b0011 ) :
                                                     4'b1111; // sw/lw

`ifdef __3STAGE__
	assign IADDR = NXPC2;
`else
    assign IADDR = NXPC;
`endif

    assign DEBUG = { RES, FLUSH, WR, RD };

`ifdef RISCV_FORMAL

    always @(posedge clk) begin
        rvfi_valid <= !RES; // && (launch_next_insn || HLT) && dbg_valid_insn;
        rvfi_order <= RES ? 0 : rvfi_order + rvfi_valid;
        rvfi_insn <= IDATA;
        rvfi_trap <= HLT;
		rvfi_halt <= HLT;
        rvfi_intr <= rvfi_pc_rdata != rvfi_pc_wdata;
        rvfi_mode <= 3;

        // care: It do not check if the instruction is valid or not
		rvfi_rs1_addr <= S1PTR;
		rvfi_rs2_addr <= S2PTR;
        rvfi_rs1_rdata <= S1PTR == 0 ? 0 : REG1[S1PTR];
        rvfi_rs2_rdata <= S2PTR == 0 ? 0 : REG2[S2PTR];

        rvfi_rd_addr = DPTR;
        rvfi_rd_wdata = DPTR == 0 ? 0 : REG1[DPTR] == REG2[DPTR] ? REG1[DPTR] : 0;
		rvfi_pc_rdata <= PC;
        rvfi_pc_wdata <= NXPC;

        rvfi_mem_addr = ;
        rvfi_rmask = ;
        rvfi_wmask = ;
        rvfi_mem_rdata = ;
        rvfi_mem_wdata = ;

        // from picoRV32 (l. 2004)
        casez (dbg_insn_opcode)
            32'b 0000000_?????_000??_???_?????_0001011: begin // getq
                rvfi_rs1_addr <= 0;
                rvfi_rs1_rdata <= 0;
            end
            32'b 0000001_?????_?????_???_000??_0001011: begin // setq
                rvfi_rd_addr <= 0;
                rvfi_rd_wdata <= 0;
            end
            32'b 0000010_?????_00000_???_00000_0001011: begin // retirq
                rvfi_rs1_addr <= 0;
                rvfi_rs1_rdata <= 0;
            end
        endcase
























        if (!resetn) begin
			rvfi_rd_addr <= 0;
			rvfi_rd_wdata <= 0;
		end else
		if (cpuregs_write && !irq_state) begin
			rvfi_rd_addr <= latched_rd;
			rvfi_rd_wdata <= latched_rd ? cpuregs_wrdata : 0;
		end else
		if (rvfi_valid) begin
			rvfi_rd_addr <= 0;
			rvfi_rd_wdata <= 0;
		end



		if (!dbg_irq_call) begin
			if (dbg_mem_instr) begin
				rvfi_mem_addr <= 0;
				rvfi_mem_rmask <= 0;
				rvfi_mem_wmask <= 0;
				rvfi_mem_rdata <= 0;
				rvfi_mem_wdata <= 0;
			end else
			if (dbg_mem_valid && dbg_mem_ready) begin
				rvfi_mem_addr <= dbg_mem_addr;
				rvfi_mem_rmask <= dbg_mem_wstrb ? 0 : ~0;
				rvfi_mem_wmask <= dbg_mem_wstrb;
				rvfi_mem_rdata <= dbg_mem_rdata;
				rvfi_mem_wdata <= dbg_mem_wdata;
			end
		end
	end

	always @* begin
		rvfi_pc_wdata = dbg_irq_call ? dbg_irq_ret : dbg_insn_addr;

		rvfi_csr_mcycle_rmask = 0;
		rvfi_csr_mcycle_wmask = 0;
		rvfi_csr_mcycle_rdata = 0;
		rvfi_csr_mcycle_wdata = 0;

		rvfi_csr_minstret_rmask = 0;
		rvfi_csr_minstret_wmask = 0;
		rvfi_csr_minstret_rdata = 0;
		rvfi_csr_minstret_wdata = 0;

		if (rvfi_valid && rvfi_insn[6:0] == 7'b 1110011 && rvfi_insn[13:12] == 3'b010) begin
			if (rvfi_insn[31:20] == 12'h C00) begin
				rvfi_csr_mcycle_rmask = 64'h 0000_0000_FFFF_FFFF;
				rvfi_csr_mcycle_rdata = {32'h 0000_0000, rvfi_rd_wdata};
			end
			if (rvfi_insn[31:20] == 12'h C80) begin
				rvfi_csr_mcycle_rmask = 64'h FFFF_FFFF_0000_0000;
				rvfi_csr_mcycle_rdata = {rvfi_rd_wdata, 32'h 0000_0000};
			end
			if (rvfi_insn[31:20] == 12'h C02) begin
				rvfi_csr_minstret_rmask = 64'h 0000_0000_FFFF_FFFF;
				rvfi_csr_minstret_rdata = {32'h 0000_0000, rvfi_rd_wdata};
			end
			if (rvfi_insn[31:20] == 12'h C82) begin
				rvfi_csr_minstret_rmask = 64'h FFFF_FFFF_0000_0000;
				rvfi_csr_minstret_rdata = {rvfi_rd_wdata, 32'h 0000_0000};
			end
		end
	end

`endif




endmodule
