/*** Core Wrapper ***/

import mrv32_pkg::*;

module mrv32_core (
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
    
    output logic unsupported_instr
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
instr_fetch fetch(.clk(clk), .rst_n(rst_n), .a_rvalid(a_rvalid), .a_valid(a_valid), .a_addr(a_addr),
                .a_wdata(a_wdata), .a_wstrb(a_wstrb), .a_rdata(a_rdata), .instr(instr_if), .pc(pc_if), .pc_next(pc_next),
                .instr_valid(instr_valid_fetch), .instr_accept(instr_accept));

logic [64:0] reg_if_id;
// Stage Register between IF and ID
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_if_id <= 0;
    else if (!stall)
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
instr_decode decode(.instr(instr), .rs1_addr(rs1_addr), .rs2_addr(rs2_addr), .rd_addr(rd_addr), .load_unsigned(load_unsigned),
                    .aluop(aluop), .alusrc(alusrc), .mem_ren(mem_ren), .mem_wen(mem_wen), .mem_wstrb(mem_wstrb), .br_sel(br_sel),
                    .reg_wen(reg_wen), .is_lui(is_lui), .is_auipc(is_auipc), .is_jalr(is_jalr), .imm(imm), .unsupported_instr(unsupported));

assign instr_valid_decode = iv_if_id & ~unsupported;
assign mem_valid = mem_ren | mem_wen; // if either load or store then it's a mem op
assign unsupported_instr = unsupported & iv_if_id;

logic load_unsigned, load_unsigned_ex, is_jalr_ex;
logic [4:0] rs1_addr_ex, rs2_addr_ex, rd_addr_ex;
logic [3:0] aluop_ex;
logic [3:0] mem_wstrb_ex;
logic alusrc_ex, mem_ren_ex, mem_wen_ex, reg_wen_ex, is_lui_ex, is_auipc_ex, mem_valid_ex, instr_valid_ex;
logic [31:0] imm_ex;

logic [98:0] reg_id_ex;
// Stage Register between ID and EX
always_ff @(posedge clk) begin
    if (!rst_n)
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


logic [31:0] rs1_data, rs2_data, op2, alu_result;

registerFile mrv_rf(.clk(clk), .rst_n(rst_n), .rs1_addr(rs1_addr_ex), .rs2_addr(rs2_addr_ex), .rd_addr(rf_waddr),
                    .rd_data(rf_wdata), .reg_wen(rf_wen), .rs1_data(rs1_data), .rs2_data(rs2_data));

assign op2 = alusrc_ex ? imm_ex : rs2_data;

/** ALU (EX) **/
alu mrv_alu(.op1(rs1_data), .op2(op2), .aluop(aluop_ex), .result(alu_result));


logic [4:0] rd_addr_mem;
logic mem_ren_mem, mem_wen_mem, mem_valid_mem, reg_wen_mem, is_lui_mem, is_auipc_mem, instr_valid_mem;
logic [3:0] mem_wstrb_mem;
logic [31:0] alu_result_mem, imm_mem, rs2_mem;
logic load_unsigned_mem, is_jalr_mem;
logic [147:0] reg_ex_mem;
// Stage Register between EX and MEM
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_ex_mem <= 0;
    else if (!stall)
        reg_ex_mem <= {br_sel_ex, is_jalr_ex, is_auipc_ex, rs2_data, pc_ex, imm_ex, instr_valid_ex,
        load_unsigned_ex, rd_addr_ex, mem_ren_ex, mem_wen_ex, mem_wstrb_ex, reg_wen_ex, is_lui_ex, mem_valid_ex,
        alu_result};
end

assign br_sel_mem = reg_ex_mem[147:146];
assign is_jalr_mem = reg_ex_mem[145];
assign is_auipc_mem = reg_ex_mem[144];
assign rs2_mem = reg_ex_mem[143:112];
assign pc_mem = reg_ex_mem[111:80];
assign imm_mem = reg_ex_mem[79:48];
assign instr_valid_mem = reg_ex_mem[47];
assign load_unsigned_mem = reg_ex_mem[46];
assign rd_addr_mem = reg_ex_mem[45:41];
assign mem_ren_mem = reg_ex_mem[40];
assign mem_wen_mem = reg_ex_mem[39];
assign mem_wstrb_mem = reg_ex_mem[38:35];
assign reg_wen_mem = reg_ex_mem[34];
assign is_lui_mem = reg_ex_mem[33];
assign mem_valid_mem = reg_ex_mem[32];
assign alu_result_mem = reg_ex_mem[31:0];


logic take_branch, take_branch_wb;
br_control mrv_bru(.br_sel(br_sel_mem), .alu_result(alu_result_mem), .take_branch(take_branch));


logic lsu_done;
logic [31:0] load_data;

/** LSU (MEM) **/
mrv32_lsu lsu(.clk(clk), .rst_n(rst_n), .b_valid(b_valid), .b_addr(b_addr), .b_wdata(b_wdata), .b_wstrb(b_wstrb),
              .b_rdata(b_rdata), .b_rvalid(b_rvalid), .mem_valid(mem_valid_mem), .mem_ren(mem_ren_mem),
              .mem_wen(mem_wen_mem), .mem_wstrb(mem_wstrb_mem), .load_unsigned(load_unsigned_mem), .eff_addr(alu_result_mem),
              .lsu_done(lsu_done), .load_data(load_data), .store_data(rs2_mem));

logic stall; // 1 => stall the pipeline

assign stall = mem_valid_mem & !lsu_done;

logic [4:0] rd_addr_wb;
logic instr_valid_wb, reg_wen_wb, is_lui_wb, is_auipc_wb, mem_ren_wb, is_jalr_wb;
logic [31:0] alu_result_wb, load_data_wb, imm_wb;

logic true_instr_valid;
assign true_instr_valid = instr_valid_mem & (!mem_valid_mem | lsu_done);

logic [139:0] reg_mem_wb;
// Stage Register between MEM and WB
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_mem_wb <= 0;
    else if (!stall)
        reg_mem_wb <= {take_branch, is_jalr_mem, is_auipc_mem, pc_mem, imm_mem, mem_ren_mem, true_instr_valid, rd_addr_mem, reg_wen_mem, is_lui_mem, alu_result_mem, load_data};
end

assign take_branch_wb = reg_mem_wb[139];
assign is_jalr_wb = reg_mem_wb[138];
assign is_auipc_wb = reg_mem_wb[137];
assign pc_wb = reg_mem_wb[136:105];
assign imm_wb = reg_mem_wb[104:73];
assign mem_ren_wb = reg_mem_wb[72];
assign instr_valid_wb = reg_mem_wb[71];
assign rd_addr_wb = reg_mem_wb[70:66];
assign reg_wen_wb = reg_mem_wb[65];
assign is_lui_wb = reg_mem_wb[64];
assign alu_result_wb = reg_mem_wb[63:32];
assign load_data_wb = reg_mem_wb[31:0];

logic [31:0] jal_target;
assign jal_target = is_jalr_wb ? (alu_result_wb & ~32'd1) : (pc_wb + imm_wb);

/** Write Back **/
mrv32_wb wb(.wb_valid(instr_valid_wb), .reg_wen_in(reg_wen_wb), .mem_ren_in(mem_ren_wb), .is_lui_in(is_lui_wb),
            .is_auipc_in(is_auipc_wb), .take_branch(take_branch_wb),.rd_addr_in(rd_addr_wb), .alu_result_in(alu_result_wb), .load_data_in(load_data_wb),
            .imm_in(imm_wb), .pc_in(pc_wb), .jal_target_in(jal_target),
            .instr_accept(instr_accept), .pc_next(pc_next), .rf_wen(rf_wen), .rf_waddr(rf_waddr), .rf_wdata(rf_wdata));

endmodule;