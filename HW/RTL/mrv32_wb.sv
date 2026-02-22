/*
 * MRV32 Writeback (WB) Stage
 *
 * For the blocking (one-instruction-in-flight) MRV32 core.
 *
 * Responsibilities:
 *   - Select writeback data (rd_data) for the register file
 *   - Gate register writes (rf_wen) using wb_valid and reg_wen_in
 *   - Generate instr_accept (commit pulse) back to instruction fetch
 *   - Compute pc_next for instruction fetch (pc+4 or jump target)
 *
 * Supported WB sources (bring-up subset):
 *   - LUI   : rd_data = imm_in
 *   - JAL   : rd_data = pc_in + 4
 *   - LOAD  : rd_data = load_data_in
 *   - ALU   : rd_data = alu_result_in
 *
 * Notes:
 *   - This module is combinational; the wrapper should latch all *_in signals.
 *   - x0 write suppression is handled by the register file, but we also gate rf_wen
 *     with (rd_addr_in != 0) here for extra safety.
 */

module mrv32_wb (
    // WB-stage valid token
    input  logic        wb_valid,

    // Decoded/latched control for this instruction
    input  logic        reg_wen_in,
    input  logic        mem_ren_in,
    input  logic        is_lui_in,
    input  logic        is_jal_in,

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

      // Next PC: sequential unless JAL
      if (is_jal_in) pc_next = jal_target_in;

      // Writeback mux
      if (is_lui_in) begin
        rf_wdata = imm_in;
      end else if (is_jal_in) begin
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