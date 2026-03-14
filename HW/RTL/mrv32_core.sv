//==============================================================================
// Module: mrv32_core v1.1
//------------------------------------------------------------------------------
// Description:
//   Single-issue RV32I core with staged pipeline registers.
//   Only one instruction is allowed in flight at a time (serialized execution).
//
//   The design maintains IF/ID/EX/MEM/WB stage registers to provide a clean
//   structural foundation for future conversion into a fully pipelined
//   5-stage processor with hazard detection and forwarding.
//
// Architecture:
//   - RV32I subset (no FENCE, ECALL, EBREAK or CSR instructions)
//   - Branch resolved in MEM stage
//   - PC updated in WB stage (single instruction in flight)
//
// Future Extensions:
//   - Enable overlapping execution (true 5-stage pipeline)
//   - Add hazard detection unit
//   - Add forwarding paths
//   - Add pipeline flush on branch
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

import mrv32_pkg::*;

module mrv32_core #(
    parameter logic [31:0] PERIPH_BASE = 32'h10000000,
    parameter logic [31:0] PERIPH_END = 32'h10001000
) (
    input logic clk,
    input logic rst_n,

    output logic                  a_valid,
    output logic [ADDR_WIDTH-1:0] a_addr,
    output logic [31:0]           a_wdata,
    output logic [3:0]            a_wstrb,
    input  logic [31:0]           a_rdata,
    input  logic                  a_rvalid,
    
    output logic                  b_valid,
    output logic [ADDR_WIDTH-1:0] b_addr,
    output logic [31:0]           b_wdata,
    output logic [3:0]            b_wstrb,
    input  logic [31:0]           b_rdata,
    input  logic                  b_rvalid,

    // Peripherals
    output logic periph_valid,
    output logic [31:0] periph_addr,
    output logic [31:0] periph_wdata,
    
    output logic illegal_instr
);

logic [31:0] instr_if, instr;
logic [31:0] pc_if, pc_id, pc_ex, pc_mem, pc_wb, pc_next;
logic instr_valid_fetch, iv_if_id, instr_valid_decode;
logic instr_accept;

// From WB back to RF
logic rf_wen;
logic [4:0] rf_waddr;
logic [31:0] rf_wdata;

/** Fetch **/
mrv32_fetch fetch(.clk(clk), .rst_n(rst_n), .a_rvalid(a_rvalid), .a_valid(a_valid), .a_addr(a_addr),
                .a_wdata(a_wdata), .a_wstrb(a_wstrb), .a_rdata(a_rdata), .instr(instr_if), .pc(pc_if),
                .instr_valid(instr_valid_fetch), .branch_target(jal_target), .take_branch(take_branch), .stall(stall | load_stall));

logic [64:0] reg_if_id;
// Stage Register between IF and ID
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_if_id <= 0;
    else if (!stall && !load_stall)
        reg_if_id <= {instr_if, pc_if, instr_valid_fetch};
end

assign instr = reg_if_id[64:33];
assign pc_id = reg_if_id[32:1];
assign iv_if_id = reg_if_id[0];


logic [4:0] rs1_addr;
logic [4:0] rs2_addr;
logic [4:0] rd_addr;
logic [3:0] aluop;
logic alusrc; // 1 => use immediate for op2

logic mem_ren, mem_wen, mem_valid;
logic [3:0] mem_wstrb;
logic reg_wen;
logic is_lui, is_auipc, is_jalr, unsupported;
logic [1:0] br_sel, br_sel_ex, br_sel_mem;

logic [31:0] imm;

/** Decode **/
mrv32_decode decode(.instr(instr), .rs1_addr(rs1_addr), .rs2_addr(rs2_addr), .rd_addr(rd_addr), .load_unsigned(load_unsigned),
                    .aluop(aluop), .alusrc(alusrc), .mem_ren(mem_ren), .mem_wen(mem_wen), .mem_wstrb(mem_wstrb), .br_sel(br_sel),
                    .reg_wen(reg_wen), .is_lui(is_lui), .is_auipc(is_auipc), .is_jalr(is_jalr), .imm(imm), .unsupported_instr(unsupported));

assign instr_valid_decode = iv_if_id & ~unsupported;
assign mem_valid = mem_ren | mem_wen; // if either load or store then it's a mem op
assign illegal_instr = unsupported & iv_if_id;

logic load_unsigned, load_unsigned_ex, is_jalr_ex;
logic [4:0] rs1_addr_ex, rs2_addr_ex, rd_addr_ex;
logic [3:0] aluop_ex;
logic [3:0] mem_wstrb_ex;
logic alusrc_ex, mem_ren_ex, mem_wen_ex, reg_wen_ex, is_lui_ex, is_auipc_ex, mem_valid_ex, instr_valid_ex;
logic [31:0] imm_ex;

logic [98:0] reg_id_ex;
// Stage Register between ID and EX
always_ff @(posedge clk) begin
    if (!rst_n || load_stall)
        reg_id_ex <= 0;
    else if (!stall)
        reg_id_ex <= {br_sel, is_jalr, is_auipc, pc_id, instr_valid_decode,
        load_unsigned, rs1_addr, rs2_addr, rd_addr, aluop, alusrc, mem_ren, mem_wen, mem_wstrb, reg_wen, is_lui, imm,
        mem_valid};
end

assign br_sel_ex = reg_id_ex[98:97];
assign is_jalr_ex = reg_id_ex[96];
assign is_auipc_ex = reg_id_ex[95];
assign pc_ex = reg_id_ex[94:63];
assign instr_valid_ex = reg_id_ex[62];
assign load_unsigned_ex = reg_id_ex[61];
assign rs1_addr_ex  = reg_id_ex[60:56];
assign rs2_addr_ex  = reg_id_ex[55:51];
assign rd_addr_ex   = reg_id_ex[50:46];
assign aluop_ex     = reg_id_ex[45:42];
assign alusrc_ex    = reg_id_ex[41];
assign mem_ren_ex   = reg_id_ex[40];
assign mem_wen_ex   = reg_id_ex[39];
assign mem_wstrb_ex = reg_id_ex[38:35];
assign reg_wen_ex   = reg_id_ex[34];
assign is_lui_ex    = reg_id_ex[33];
assign imm_ex       = reg_id_ex[32:1];
assign mem_valid_ex = reg_id_ex[0];


logic [31:0] rs1_data, rs2_data, op2, op1, alu_result;

mrv32_regfile mrv_rf(.clk(clk), .rst_n(rst_n), .rs1_addr(rs1_addr_ex), .rs2_addr(rs2_addr_ex), .rd_addr(rf_waddr),
                    .rd_data(rf_wdata), .reg_wen(rf_wen), .rs1_data(rs1_data), .rs2_data(rs2_data));

// RS1 may be forwarded from EX or MEM
assign op1 = is_lui_ex  ? 32'd0   :
             is_auipc_ex ? pc_ex  :
                           rs1_fwd;

// RS2 may be forwared from EX or MEM
assign op2 = (is_lui_ex | is_auipc_ex) ? imm_ex :
             alusrc_ex                 ? imm_ex  :
                                         rs2_fwd;

/** ALU (EX) **/
mrv32_alu mrv_alu(.op1(op1), .op2(op2), .aluop(aluop_ex), .result(alu_result));


logic [4:0] rd_addr_mem;
logic mem_ren_mem, mem_wen_mem, mem_valid_mem, reg_wen_mem, instr_valid_mem;
logic [3:0] mem_wstrb_mem;
logic [31:0] alu_result_mem, imm_mem, rs2_mem;
logic load_unsigned_mem, is_jalr_mem;
logic [145:0] reg_ex_mem;
// Stage Register between EX and MEM
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_ex_mem <= 0;
    else if (!stall)
        reg_ex_mem <= {br_sel_ex, is_jalr_ex, rs2_fwd, pc_ex, imm_ex, instr_valid_ex,
        load_unsigned_ex, rd_addr_ex, mem_ren_ex, mem_wen_ex, mem_wstrb_ex, reg_wen_ex, mem_valid_ex,
        alu_result};
end

assign br_sel_mem = reg_ex_mem[145:144];
assign is_jalr_mem = reg_ex_mem[143];
assign rs2_mem = reg_ex_mem[142:111];
assign pc_mem = reg_ex_mem[110:79];
assign imm_mem = reg_ex_mem[78:47];
assign instr_valid_mem = reg_ex_mem[46];
assign load_unsigned_mem = reg_ex_mem[45];
assign rd_addr_mem = reg_ex_mem[44:40];
assign mem_ren_mem = reg_ex_mem[39];
assign mem_wen_mem = reg_ex_mem[38];
assign mem_wstrb_mem = reg_ex_mem[37:34];
assign reg_wen_mem = reg_ex_mem[33];
assign mem_valid_mem = reg_ex_mem[32];
assign alu_result_mem = reg_ex_mem[31:0];


logic take_branch, take_branch_wb;
mrv32_bru mrv_bru(.br_sel(br_sel_mem), .alu_result(alu_result_mem), .take_branch(take_branch));


// Jump target, wired from MEM back to Fetch
logic [31:0] jal_target;
assign jal_target = is_jalr_mem ? (alu_result_mem & ~32'd1) : (pc_mem + imm_mem);

logic lsu_done;
logic [31:0] load_data;

/** LSU (MEM) **/
mrv32_lsu lsu(.clk(clk), .rst_n(rst_n), .b_valid(b_valid), .b_addr(b_addr), .b_wdata(b_wdata), .b_wstrb(b_wstrb),
              .b_rdata(b_rdata), .b_rvalid(b_rvalid), .mem_valid(mem_valid_mem), .mem_ren(mem_ren_mem),
              .mem_wen(mem_wen_mem), .mem_wstrb(mem_wstrb_mem), .load_unsigned(load_unsigned_mem), .eff_addr(alu_result_mem),
              .lsu_done(lsu_done), .load_data(load_data), .store_data(rs2_mem));

/** Peripheral Decoder (within MEM stage) **/
mrv32_periph_decoder #(
    .PERIPH_BASE(PERIPH_BASE),
    .PERIPH_END (PERIPH_END)
) periph_dec (
    .mem_wen    (mem_wen_mem & lsu_done),
    .eff_addr   (alu_result_mem),
    .store_data (rs2_mem),
    .periph_valid(periph_valid),
    .periph_addr (periph_addr),
    .periph_wdata(periph_wdata)
);


logic stall; // 1 => stall the pipeline

assign stall = mem_valid_mem & !lsu_done;

logic [4:0] rd_addr_wb;
logic instr_valid_wb, reg_wen_wb, mem_ren_wb;
logic [31:0] alu_result_wb, load_data_wb;

logic true_instr_valid;
assign true_instr_valid = instr_valid_mem & (!mem_valid_mem | lsu_done);

logic [104:0] reg_mem_wb;
// Stage Register between MEM and WB
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_mem_wb <= 0;
    else if (!stall)
        reg_mem_wb <= {take_branch, pc_mem, mem_ren_mem, true_instr_valid, rd_addr_mem, reg_wen_mem, alu_result_mem, load_data};
end

assign take_branch_wb = reg_mem_wb[104];
assign pc_wb = reg_mem_wb[103:72];
assign mem_ren_wb = reg_mem_wb[71];
assign instr_valid_wb = reg_mem_wb[70];
assign rd_addr_wb = reg_mem_wb[69:65];
assign reg_wen_wb = reg_mem_wb[64];
assign alu_result_wb = reg_mem_wb[63:32];
assign load_data_wb = reg_mem_wb[31:0];

// instr_accept: debug signal, to count accepted instructions. TODO: Add CSR with minstret and wire this there instead

/** Write Back **/
mrv32_wb wb(.wb_valid(instr_valid_wb), .reg_wen_in(reg_wen_wb), .mem_ren_in(mem_ren_wb), .take_branch(take_branch_wb),
            .rd_addr_in(rd_addr_wb), .alu_result_in(alu_result_wb), .load_data_in(load_data_wb),
            .pc_in(pc_wb), .instr_accept(instr_accept), .rf_wen(rf_wen), .rf_waddr(rf_waddr), .rf_wdata(rf_wdata));


logic fwd_rs1_ex, fwd_rs2_ex, fwd_rs1_mem, fwd_rs2_mem, load_stall;

/** Hazard Detection Unit **/
mrv32_hzdu hzdu (
    .reg_wen_ex  (reg_wen_mem),    // producer one stage ahead of EX = MEM
    .rd_ex       (rd_addr_mem),
    .reg_wen_mem (reg_wen_wb),     // producer two stages ahead = WB
    .rd_mem      (rd_addr_wb),
    .rs1_id      (rs1_addr_ex),    // consumer in EX
    .rs2_id      (rs2_addr_ex),    // consumer in EX
    .rs2_id_used (!alusrc_ex | mem_wen_ex),
    .mem_ren_ex  (mem_ren_mem),
    .fwd_rs1_ex  (fwd_rs1_ex),
    .fwd_rs2_ex  (fwd_rs2_ex),
    .fwd_rs1_mem (fwd_rs1_mem),
    .fwd_rs2_mem (fwd_rs2_mem),
    .load_stall  (load_stall)
);

/** Forwarding Connections **/
logic [31:0] rs1_fwd, rs2_fwd, fwd_val_ex, fwd_val_mem;

// fwd_rs1_ex => producer is in MEM => use br_sel_mem
assign fwd_val_ex  = (br_sel_mem == BR_ALWAYS) ? pc_mem + 32'd4 : alu_result_mem;

// fwd_rs1_mem => producer is in WB => rf_wdata already correct
assign fwd_val_mem = rf_wdata;

assign rs1_fwd = fwd_rs1_ex  ? fwd_val_ex  :
                 fwd_rs1_mem ? fwd_val_mem  :
                               rs1_data;

assign rs2_fwd = fwd_rs2_ex  ? fwd_val_ex  :
                 fwd_rs2_mem ? fwd_val_mem  :
                               rs2_data;

endmodule;