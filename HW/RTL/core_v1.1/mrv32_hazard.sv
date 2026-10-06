//==============================================================================
// Module: mrv32_hazard v1.1
//------------------------------------------------------------------------------
// Description:
//   Hazard detection unit for MRV32.
//
// Responsibilities:
//  - Detect data hazards between EX/MEM and ID stages
//
// Output:
//   - Control signals for EX, MEM, WB
//
// Version v1.1 notes:
//   Designed to be compatible with fully pipelined operation.
//
// Author: Martim Bento
// Date  : 06/10/2026
//==============================================================================

module mrv32_hzdu (
    input logic reg_wen_ex,
    input logic [4:0] rd_ex, // alu_result_ex
    input logic reg_wen_mem,
    input logic [4:0] rd_mem, // alu_result_mem
    input logic [4:0] rs1_id,
    input logic [4:0] rs2_id,
    input logic rs2_id_used,
    input logic mem_ren_ex,
    output logic fwd_rs1_ex,
    output logic fwd_rs2_ex,
    output logic fwd_rs1_mem,
    output logic fwd_rs2_mem,
    output logic load_stall
);

assign fwd_rs1_ex = (rd_ex == rs1_id) & reg_wen_ex & (rd_ex != 0);
assign fwd_rs2_ex = (rd_ex == rs2_id) & reg_wen_ex & (rd_ex != 0) & rs2_id_used;

assign fwd_rs1_mem = (rd_mem == rs1_id) & reg_wen_mem & (rd_mem != 0);
assign fwd_rs2_mem = (rd_mem == rs2_id) & reg_wen_mem & (rd_mem != 0) & rs2_id_used;

assign load_stall = mem_ren_ex & ((rd_ex == rs1_id) | ((rd_ex == rs2_id) & rs2_id_used)) & (rd_ex != 0);

endmodule