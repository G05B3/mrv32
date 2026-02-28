//==============================================================================
// Module: mrv32_wb v1.0
//------------------------------------------------------------------------------
// Description:
//   Writeback stage of the RV32I core.
//
// Responsibilities:
//   - Select final writeback data (ALU, memory, immediate, PC+4)
//   - Generate register file write enable and data
//   - Compute next PC value
//   - Assert instr_accept to allow next instruction fetch
//
// Notes:
//   PC update occurs in this stage.
//   Only one instruction is retired at a time in bring-up mode.
//   Designed to support future pipelined execution.
//
// Author: Martim Bento
// Date  : 28/02/2026
//==============================================================================

module mrv32_wb (
    // WB-stage valid token
    input  logic        wb_valid,

    // Decoded/latched control for this instruction
    input  logic        reg_wen_in,
    input  logic        mem_ren_in,
    input  logic        is_lui_in,
    input  logic        is_auipc_in,
    input  logic        take_branch,

    input  logic [4:0]  rd_addr_in,

    // Latched data inputs
    input  logic [31:0] alu_result_in,
    input  logic [31:0] imm_in,
    input  logic [31:0] load_data_in,
    input  logic [31:0] pc_in,         // PC of the retiring instruction
    input  logic [31:0] jal_target_in,  // already computed target (pc + imm) in wrapper/EX

    // Outputs to register file write port
    output logic        rf_wen,
    output logic [4:0]  rf_waddr,
    output logic [31:0] rf_wdata,

    // Outputs to fetch / core control
    output logic        instr_accept,   // commit pulse
    output logic [31:0] pc_next         // next PC on commit
);

  // default outputs
  always_comb begin
    rf_wen      = 1'b0;
    rf_waddr    = rd_addr_in;
    rf_wdata    = 32'd0;

    instr_accept = 1'b0;
    pc_next      = pc_in + 32'd4;

    if (wb_valid) begin
      // Commit this instruction (blocking core)
      instr_accept = 1'b1;

      // Next PC: sequential unless JAL or JALR
      if (take_branch) pc_next = jal_target_in;

      // Writeback mux
      if (is_lui_in) begin
        rf_wdata = imm_in;
      end else if (is_auipc_in) begin
        rf_wdata = imm_in + pc_in;
      end else if (take_branch) begin
        rf_wdata = pc_in + 32'd4;
      end else if (mem_ren_in) begin
        rf_wdata = load_data_in;
      end else begin
        rf_wdata = alu_result_in;
      end

      // Register write enable gating
      rf_wen = reg_wen_in && (rd_addr_in != 5'd0);
    end
  end

endmodule